----------------------------------------------------------------------------------
-- This module is for interfacing for the wiznet5500 spi-ethernet chip
-- 
-- 
-- The SPI Frameformat is specified on page 15 of the datasheet and is as follows
-- 2 bytes : Offset address
-- 1 byte : control byte, [7:3] = block select bits, [2] = R/W , [1:0] = SPI_OP_mode
-- N data bytes
--
-- The block select bits select the block to which the offset address belongs.

-- The device contains one common register block, 8 socket register blocks and a TX 
-- and RX buffer block for each socket. Each of these these is referenced with a
-- different combination of block select bits, constants are defined for these memory locations
-- further down in the module.
----------------------------------------------------------------------------------


--PACKAGE THESE TYPES SO TESTBENCH CAN USE THEM


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.Config.all;
use ieee.math_real.all;
use work.pkt_types_pkg.all;
use work.eth_pkg.all;
use work.network_config.all;



entity w5500_spi_ethernet is 
generic (
    clk_freq : integer := 100_000_000;
    spi_freq : integer := 15_000_000;
    max_packet_size_bytes : integer range 0 to MAX_ETH_DATA_SIZE := 100;
);
port (
    reset : in std_logic;
    clk : in std_logic;
    new_data_avail : in std_logic;
    
    --spi
    miso : in std_logic;
    int : in std_logic; --interupt pin for the wiznet module to signal data recieved
    mosi : out std_logic;
    ss : out std_logic;
    sck : out std_logic;

    --signals that the new_data_avail_signal can be deasserted and data_in changed, as the module has latched the input data
    ack_new_data : out std_logic; 

    data_in : in std_logic_vector(max_packet_size_bytes*BYTE -1 downto 0); --hex data to send over eth
    data_size_bytes : in natural range 0 to max_packet_size_bytes;

    pkt_id_out : out pkt_type_t := UNKNOWN; --id of command recieved over eth
    pkt_data_out : out std_logic_vector(max_packet_data_bytes*BYTE -1 downto 0); --data of command recieved over eth

    --DEBUG SIGNALS FOR TESTBENCH (SURELY THERE IS A BETTER WAY TO EXPOSE INTERNAL DATA TO THAN THIS
    -- modelsim has a way of "spying" however the vivado simulator does not)
    -- https://stackoverflow.com/questions/35189588/vhdl-testbench-internal-signals
    -- https://support.xilinx.com/s/question/0D52E00006hpbfKSAQ/monitor-signal-in-test-bench-of-the-unit-under-test?language=en_US
    write_ptr : out std_logic_vector(15 downto 0);
    size_recieved : out std_logic_vector(15 downto 0) := x"FFFF";
    pkt_id_bits : out std_logic_vector(7 downto 0);
    state_d : out eth_state_t;
    tx_state_d : out eth_tx_state_t;
    rx_state_d : out eth_rx_state_t    
);
end w5500_spi_ethernet;

architecture behavioural of max_ethernet is

    
    signal state : eth_state_t := SETUP;

    signal rx_state : eth_rx_state_t  := CLEAR_INTERRUPT;

   
    signal tx_state : eth_tx_state_t := READ_WRITE_PTR;

    constant BYTE : natural := 8;
---------------------------------------------------NETWORKING INFORMATION------------------------------------------------
    

    -- GAR (Gateway IP Address Register) [R/W] [0x0001 ? 0x0004] [0x00]
    -- GAR configures the default gateway address. (from datasheet)
    type GATEWAY_ADDRESS_REGISTER_ARRAY is array (0 to 3) of std_logic_vector(15 downto 0);
    constant GATEWAY_ADDRESS_REGISTERS : GATEWAY_ADDRESS_REGISTER_ARRAY := (
        x"0001",
        x"0002",
        x"0003",
        x"0004"
    );

    
    -- SUBR (Subnet Mask Register) [R/W] [0x0005 ? 0x0008] [0x00]
    -- SUBR configures the subnet mask address. (from datasheet)
    type SUBNET_MASK_REGISTER_ARRAY is array (0 to 3) of std_logic_vector(15 downto 0);
    constant SUBNET_MASK_ADDRESS_REGISTERS : SUBNET_MASK_REGISTER_ARRAY := (
        x"0005",
        x"0006",
        x"0007",
        x"0008"
    );

    -- SHAR (Source Hardware Address Register) [R/W] [0x0009 ? 0x000E] [0x00]
    -- SHAR configures the source hardware address.
    -- Ex) In case of ?00.08.DC.01.02.03?
    type MAC_ADDRESS_REGISTER_ARRAY is array (0 to 5) of std_logic_vector(15 downto 0);
    constant MAC_ADDRESS_REGISTERS : MAC_ADDRESS_REGISTER_ARRAY := (
        x"0009",
        x"000A",
        x"000B",
        x"000C",
        x"000D",
        x"000E"
    );


    type DEST_MAC_ADDRESS_REGISTER_ARRAY is array (0 to 5) of std_logic_vector(15 downto 0);
    constant DEST_MAC_ADDRESS_REGISTERS : DEST_MAC_ADDRESS_REGISTER_ARRAY := (
        x"0006",
        x"0007",
        x"0008",
        x"0009",
        x"000A",
        x"000B"
    );

    -- SIPR (Source IP Address Register) [R/W] [0x000F ? 0x0012] [0x00]
    -- SIPR configures the source IP address.
    type IP_ADDRESS_REGISTER_ARRAY is array (0 to 3) of std_logic_vector(15 downto 0);
    constant IP_ADDRESS_REGISTERS : IP_ADDRESS_REGISTER_ARRAY := (
        x"000F",
        x"0010",
        x"0011",
        x"0012"
    );

    -- DIPR configures the dest IP address.
    type DEST_IP_ADDRESS_REGISTER_ARRAY is array (0 to 3) of std_logic_vector(15 downto 0);
    constant DEST_IP_ADDRESS_REGISTERS : DEST_IP_ADDRESS_REGISTER_ARRAY := (
        x"000C",
        x"000D",
        x"000E",
        x"000F"
    );

    -------------------------------------------------Common Interrupt registers--------------------------------------------------
    constant INT_REGISTER : std_logic_vector(15 downto 0) := x"0015";
    constant INT_MASK_REGISTER : std_logic_vector(15 downto 0) := x"0016";
    constant SOCKET_INT_REGISTER : std_logic_vector(15 downto 0) := x"0017";
    constant SOCKET_INT_MASK_REGISTER : std_logic_vector(15 downto 0) := x"0018";
    constant INT_MASK : std_logic_vector(7 downto 0) := x"00";

    -------------------------------------------------Socket register fields--------------------------------------------------
    
    -- Sn_MR (Socket n Mode Register) [R/W] [0x0000] [0x00]
    -- Sn_MR configures the option or protocol type of Socket n.
    constant SnMR_REGISTER : std_logic_vector(15 downto 0) := x"0000";
    constant SnMR_MODE : std_logic_vector(7 downto 0) := x"02"; -- FOR UDP 


    -- Sn_PORT (Socket n Source Port Register) [R/W] [0x0004-0x0005] [0x0000]
    -- Sn_PORT configures the source port number of Socket n. It is valid when Socket n is
    -- used in TCP/UDP mode. It should be set before OPEN command is ordered.
    -- Ex In case of Socket 0 Port = 8888(0x22B8), configure as below,
    type S0PORT_ADDRESS_REGISTER_ARRAY is array (0 to 1) of std_logic_vector(15 downto 0);
    constant S0PORT_ADDRESS_REGISTERS : S0PORT_ADDRESS_REGISTER_ARRAY := (
        x"0004",
        x"0005"
    );

    -- Sn_DPORT (Socket n Destination Port Register) [R/W] [0x0010-0x0011] [0x00]
    -- Sn_DPORT configures or indicates the destination port number of Socket n. It is valid when Socket n is used in TCP/UDP mode.
    -- In TCP client mode, it configures the listen port number of ?TCP server? before CONNECT command.
    -- In TCP server mode, it indicates the port number of ?TCP client? after successfully establishing connection.
    -- In UDP mode, it configures the port number of peer to be transmitted the UDP packet by SEND/SEND_MAC command.
    -- Ex In case of Socket 0 Destination Port = 5001(0x1389), configure as below,
    constant DEST_S0PORT_ADDRESS_REGISTERS : S0PORT_ADDRESS_REGISTER_ARRAY := (
        x"0010",
        x"0011"
    );

    -----------------Constants for control bytes for reading and writing to different register blocks/memory locationsx ----------------------------------

    -- The device has 8 different sockets which are configurable each with identical registers allocated to them 
    -- Which socket you are addressing is determined by the first 5 bits of the SPI control byte(MSB first)  (3rd byte in the transfer)
    -- This byte also control wether you are reading or writing from the socket register. The 6th bit specifies this.

    --the 3rd byte of the spi transfer specifying a read or a write and which block the address_offset is for
    --block can be common register, socket register , or tx or rx buffer for a given socket
    constant COMMON_REGISTER_CONTROL_WRITE_BYTE : std_logic_vector(7 downto 0) := x"04";
    constant COMMON_REGISTER_CONTROL_READ_BYTE : std_logic_vector(7 downto 0) := x"00";

    constant S0_CONTROL_WRITE : std_logic_vector(7 downto 0) := x"0C";
    constant S0_CONTROL_READ : std_logic_vector(7 downto 0) := x"08";
    constant S0_TX_BUF_WRITE : std_logic_vector(7 downto 0) := X"14"; --000

    constant S0_RX_BUF_READ : std_logic_vector(7 downto 0) := X"18"; --00011
    

    --currently unused sockets
    constant S1_CONTROL_WRITE : std_logic_vector(7 downto 0) := x"2C";
    constant S1_CONTROL_READ  : std_logic_vector(7 downto 0) := x"28";

    constant S2_CONTROL_WRITE: std_logic_vector(7 downto 0) := x"4C";
    constant S2_CONTROL_READ  : std_logic_vector(7 downto 0) := x"48";

    constant S3_CONTROL_WRITE : std_logic_vector(7 downto 0) := x"6C";
    constant S3_CONTROL_READ  : std_logic_vector(7 downto 0) := x"68";

    constant S4_CONTROL_WRITE : std_logic_vector(7 downto 0) := x"8C";
    constant S4_CONTROL_READ  : std_logic_vector(7 downto 0) := x"88";
    
    constant S5_CONTROL_WRITE : std_logic_vector(7 downto 0) := x"AC";
    constant S5_CONTROL_READ : std_logic_vector(7 downto 0) := x"A8";

    constant S6_CONTROL_WRITE : std_logic_vector(7 downto 0) := x"CC";
    constant S6_CONTROL_READ  : std_logic_vector(7 downto 0) := x"C8";

    constant S7_CONTROL_WRITE : std_logic_vector(7 downto 0) := x"EC";
    constant S7_CONTROL_READ  : std_logic_vector(7 downto 0) := x"E8";



-----------------------------------------------------------SOCKET N registers and their commands/write values------------------------------------


    -- Datasheet fig 20
    -- Address of socket N command register, the value stored in this register
    -- Setting this register sets the command for the socket
    -- Commands include: OPEN,CLOSE, CONNECT, LISTEN, SEND, and RECIEVE
    -- After the command is sent to the register it is automatically cleared
    -- and the register is zeroed. T
    constant SnCR_REGISTER : std_logic_vector(15 downto 0) := x"0001";

    --Address of the write pointer reg for tx buffer belonging to socket N. (N specified by the control byte)
    constant SnTx_WR_PTR_ADRR : std_logic_vector(15 downto 0) := x"0024";
    constant SnRx_RECIEVED_SIZE_REG : std_logic_vector(15 downto 0) := x"0026";
    constant SnRx_RD_PTR_REG : std_logic_vector(15 downto 0) := x"0028";

    --interrupt register for socket n
    constant Sn_INT_REGISTER : std_logic_vector(15 downto 0) := x"0002"; 

    --int_mask_register
    constant Sn_INT_MASK_REGISTER : std_logic_vector(15 downto 0) := x"002C";

    --int mask such that the only thing that causes the interrupts from Sn is reception of data
    constant Sn_RX_ONLY_INT_MASK : std_logic_vector(7 downto 0) := x"04"; 


    --Control register commands for a socket N. (N specified by the control byte)
    constant SnCR_OPEN_SOCKET : std_logic_vector(7 downto 0) := x"01";
    constant SnCR_SEND        : std_logic_vector(7 downto 0) := x"20";
    constant SnCR_RECV        : std_logic_vector(7 downto 0) := x"40";


    constant TX_BUFFER_SIZE_ADDR : std_logic_vector(15 downto 0) := x"001F";
    constant TX_BUFFER_SIZE : std_logic_vector(7 downto 0) := x"10"; --16 kB. do not assign more.

    constant RX_BUFFER_SIZE_REG : std_logic_vector(15 downto 0) := x"001e";
    constant RX_BUFFER_SIZE : std_logic_vector(7 downto 0) := x"10";

    
    constant PHY_CONFIGURATION_REGISTER : std_logic_vector(15 downto 0) := x"002E";
    constant SET_PHY_OPERATION_MODE : std_logic_vector(7 downto 0) := x"D8";  
    

    --command to send s0's tx_buffer to the computer
    constant SEND_TO_COMPUTER : std_logic_vector(31 downto 0) :=  SnCR_REGISTER & S0_CONTROL_WRITE & SnCR_SEND;
    constant RECV_COMMAND : std_logic_vector(31 downto 0) := SnCR_REGISTER & S0_CONTROL_WRITE & SnCR_RECV;
    constant hello_str : std_logic_vector(39 downto 0) := X"68656C6C6F"; --hello

-----------------------------------------------------SPI_SIGNALS-------------------------------------------------------
    
    --SPI WIDTHS 
    constant spi_tx_reg_width_bytes : integer := max_packet_size_bytes + 3; --3 for addressing and ctrl
    constant spi_rx_reg_width_bytes : integer := pkt_id_field_size_bytes + max_packet_data_bytes;
    constant spi_tx_reg_width_bits : integer := (spi_tx_reg_width_bytes*BYTE);
    constant spi_rx_reg_width_bits : integer := (spi_rx_reg_width_bytes*BYTE);
    constant spi_tx_reg_MSB : integer := spi_tx_reg_width_bits-1;
    constant spi_rx_reg_MSB : integer := spi_rx_reg_width_bits-1;


    --SPI SIGNALS
    signal spi_tx_data_in : std_logic_vector(spi_tx_reg_MSB downto 0) := (others =>'0');
    signal spi_rx_data : std_logic_vector(spi_rx_reg_MSB downto 0) := (others=>'0');
    signal spi_bytes_to_send : integer;
    signal spi_enable : std_logic := '0';
    signal spi_active : std_logic := '0'; 
        
  
-----------------------------------------------------------SIGNALS---------------------------------------------------

    --The write pointer for socket zeroes tx buffer 
    signal s0_tx_write_ptr : std_logic_vector(15 downto 0) := (others => '0');
    
    --The rx read_ptr
    signal s0_rx_read_ptr : std_logic_vector(15 downto 0) :=  (others => '0');

    --the data contained in the last recieved packet
    signal rx_pkt_data : std_logic_vector(max_packet_data_bytes*BYTE -1 downto 0);
    --the id of the last recieved rx packet
    signal pkt_id : pkt_type_t;


    signal count : integer := 0;
    signal setup_count : integer := 0;
    signal setup_done : std_logic := '0';

begin
    --testbench mappings
    write_ptr <= s0_tx_write_ptr;
    state_d <= state;
    tx_state_d <= tx_state;
    rx_state_d <= rx_state;

    --SPI
    buffered_spi_inst : ENTITY work.buffered_spi
    generic map (CLK_FREQ => clk_freq,
            SPI_FREQ => spi_freq,
            CPHA =>  '0',
            CPOL => '0',
            -- determines how many byte can be passed at once to the SPI module
            -- for writing
            spi_tx_reg_width_bytes => spi_tx_reg_width_bytes,
            spi_rx_reg_width_bytes => spi_rx_reg_width_bytes
            )
    port map (
            clk => clk,
            reset => reset,
            spi_miso => miso,
            spi_mosi => mosi,
            spi_sck => sck,
            spi_ss => ss,

            enable => spi_enable,

            spi_tx_data_in  => spi_tx_data_in,
            spi_rx_data_out => spi_rx_data ,

            --determines how many times to shift the spi_tx_vector.
            bytes_to_send => spi_bytes_to_send,

            --wether SPI is currnetly transmittiing
            spi_active => spi_active
        );

    


    ethernet_stuff : process(clk)
        variable pkt_data_size_bytes : integer; --stores amount of bytes corresponding to the recieved pkt type
    begin
        if reset = '1' then
            state <= SETUP;
        elsif rising_edge(clk) then
            --module initialisation delay
            if count < clk_freq/10000 then 
                count <= count + 1;
            --SPI is active, wait
            elsif spi_active = '1' then
                spi_enable <= '0';
            --spi is inactive and we have not already asserted the enable
            elsif spi_active = '0' and spi_enable = '0' then
                case(state) is
                    when SETUP =>
                        spi_enable <= '1';
                        setup_count <= setup_count + 1;
                        spi_tx_data_in <= (others => '0');
                        case(setup_count) is
                            --set phy operation mode
                            when 0  => 
                            spi_bytes_to_send <= 4; 
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 4*BYTE) <= PHY_CONFIGURATION_REGISTER & COMMON_REGISTER_CONTROL_WRITE_BYTE & SET_PHY_OPERATION_MODE;

                            --set tx buffer size
                            when 1  => 
                            spi_bytes_to_send <= 4; 
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 4*BYTE) <= TX_BUFFER_SIZE_ADDR & S0_CONTROL_WRITE & TX_BUFFER_SIZE;
                            
                            --mac addr
                            when 2  =>
                            spi_bytes_to_send <= 9;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 9*BYTE)<= MAC_ADDRESS_REGISTERS(0) & COMMON_REGISTER_CONTROL_WRITE_BYTE & MAC_ADDRESS(47 downto 0);
                            
                            
                            --ip addr
                            when 3  => 
                            spi_bytes_to_send <= 7;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 7*BYTE) <= IP_ADDRESS_REGISTERS(0) & COMMON_REGISTER_CONTROL_WRITE_BYTE & IP_ADDRESS(31 downto 0);

    
                            -- GATEWAY_ADDR
                            when 4 => 
                            spi_bytes_to_send <= 7;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 7*BYTE) <= GATEWAY_ADDRESS_REGISTERS(0) & COMMON_REGISTER_CONTROL_WRITE_BYTE & GATEWAY_IP_ADDRESS(31 downto 0);
                            
                           
                            --subnet mask
                            when 5 => 
                            spi_bytes_to_send <= 7;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 7*BYTE) <= SUBNET_MASK_ADDRESS_REGISTERS(0) & COMMON_REGISTER_CONTROL_WRITE_BYTE & SUBNET_MASK_ADDRESS(31 downto 0);
        
                            --choose UDP
                            when 6=> 
                            spi_bytes_to_send <= 4;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 4*BYTE) <= SnMR_REGISTER & S0_CONTROL_WRITE & SnMR_MODE;
                            
                            --S0 PORT #
                            when 7 => 
                            spi_bytes_to_send <=5;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 5*BYTE) <= S0PORT_ADDRESS_REGISTERS(0) & S0_CONTROL_WRITE & S0PORT_NUMBER(15 downto 0);

                            --DONT NEED AS THE WIZNET IMPLEMENTS ARP
                            --DEST MAC ADDR
                            --when 8 => 
                            --spi_bytes_to_send <= 9;
                            --spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 9*BYTE) <= DEST_MAC_ADDRESS_REGISTERS(0) & S0_CONTROL_WRITE & DEST_MAC_ADDRESS(47 downto 0);
                            
                            --dest ip 
                            when 8 => 
                            spi_bytes_to_send <= 7;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 7*BYTE) <= DEST_IP_ADDRESS_REGISTERS(0) & S0_CONTROL_WRITE & DEST_IP_ADDRESS(31 downto 0);
                            
    
                            --DEST port #
                            when 9 => 
                            spi_bytes_to_send <= 5;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 5*BYTE) <= DEST_S0PORT_ADDRESS_REGISTERS(0) & S0_CONTROL_WRITE & DEST_S0PORT_NUMBER(15 downto 0);

                            --enable interrupts from socket 0
                            when 10=>
                            spi_bytes_to_send <= 4;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 4*BYTE) <= SOCKET_INT_MASK_REGISTER & COMMON_REGISTER_CONTROL_WRITE_BYTE & x"01";

                            --mask socketn interrupts as we only care interrupt due to recived packets
                            when 11 =>
                            spi_bytes_to_send <= 4;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 4*BYTE) <=  Sn_INT_MASK_REGISTER & S0_CONTROL_WRITE & Sn_RX_ONLY_INT_MASK;
                            

                            when 12=>
                            spi_bytes_to_send <= 4;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 4*BYTE) <=  INT_MASK_REGISTER & COMMON_REGISTER_CONTROL_WRITE_BYTE & x"00"; --dest unreachable interrupt
                            
                            --set rx buffer size
                            when 13 =>
                            spi_bytes_to_send <=4;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 4*BYTE)  <= RX_BUFFER_SIZE_REG & S0_CONTROL_WRITE  & RX_BUFFER_SIZE;

                            --open socket
                            when 14 => 
                            spi_bytes_to_send <= 4;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 4*BYTE) <= SnCR_REGISTER & S0_CONTROL_WRITE & SnCR_OPEN_SOCKET;

                            --intialise the read ptr
                            when 15 =>
                            spi_bytes_to_send <= 5;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 5*BYTE) <= SnRx_RD_PTR_REG & S0_CONTROL_READ & x"0000";

                            when 16 =>
                            s0_rx_read_ptr <= spi_rx_data(15 downto 0);
                            
                            --init finished move_to_dile
                            when others => 
                                state <= IDLE;
                                spi_bytes_to_send <= 0;
                                spi_enable <= '0';
                        end case;
                             
                    
                    when IDLE =>
                        
                        spi_tx_data_in <= (others => '0');
                        if INT = '0' then --interupt asserted data ready to receieve
                            spi_enable <= '1';
                            state <= RECIEVING;
                            --GET THE SIZE RECIEVED
                            spi_bytes_to_send <= 5;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 5*BYTE) <= SnRx_RECIEVED_SIZE_REG & S0_CONTROL_READ & x"0000";
                            rx_state <= CLEAR_INTERRUPT;
                        elsif new_data_avail = '1' then  -- WHEN NEW DATA AVAILABLE move to send state
                            state <= SENDING;
                            tx_state <= READ_WRITE_PTR;
                        end if;

                        
                    when SENDING =>
                        spi_enable <= '1';
                        spi_tx_data_in <= (others => '0');
                        
                        case (tx_state) is
                            --READING write pointer REALLY SHOULDN'T BE NECCECARY AS WE should just keep track of the state of the pointer ourselves.
                            --however for consistency's sake, in accordance with the datasheet we will do it
                            when READ_WRITE_PTR =>                               
                                spi_bytes_to_send <= 5;
                                spi_tx_data_in(spi_tx_reg_MSB downto spi_tx_reg_width_bits - 5*BYTE) <=  SnTx_WR_PTR_ADRR & S0_CONTROL_READ & x"0000";    
                                tx_state <= WRITE_BUFF;
                            when WRITE_BUFF =>
                                --write pointer has ben read into spi_rx_data, latch into the s0_tx_write_ptr reg
                                s0_tx_write_ptr <= spi_rx_data(15 downto 0);   

                                --send the data to the tx buffer
                                spi_bytes_to_send <= 3+ data_size_bytes                           
                                spi_tx_data_in <= s0_tx_write_ptr & S0_TX_BUF_WRITE & data_in;

                                --have read in data to the spi_tx_data_in reg, can signal that the data_in_signal is now safe to change 
                                --signals that the new_data_avail_signal can be deasserted, as the module has read the input data
                                ack_new_data <= '1';  
                                
                                --the write pointer needs to be updated by the amount of bytes we have read
                                s0_tx_write_ptr <=  std_logic_vector(unsigned(s0_tx_write_ptr) + to_unsigned(max_packet_size_bytes, 16));
                                tx_state <= WRITE_WRITE_PTR; 
                            when WRITE_WRITE_PTR =>

                                ack_new_data <= '0'; --deassert signal as it should have been detected

                                --update the write ptr register in the w5500
                                spi_bytes_to_send <= 5;
                                spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 5*BYTE) <=  SnTx_WR_PTR_ADRR & S0_CONTROL_WRITE & s0_tx_write_ptr;
                                tx_state <= send;
                            when send =>
                                --Issue a send command so the w5500 sends what it's in the tx_buffer over ethernet.
                                spi_bytes_to_send <= 4;
                                spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 4*BYTE) <=  SEND_TO_COMPUTER;
                                tx_state <= READ_WRITE_PTR;
                                state <= IDLE;
                        end case;

                    when RECIEVING =>
                    spi_enable <= '1';
                    case (rx_state) is 
                        --clear interrupt
                        when CLEAR_INTERRUPT =>
                            size_recieved <= spi_rx_data(15 downto 0); --load the size recieved var

                            --send the command to clear the interrupt
                            spi_bytes_to_send <= 4;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 4*BYTE) <=  Sn_INT_REGISTER & S0_CONTROL_WRITE & x"FF";
                            rx_state <= READ_ID;
                                           
                        when READ_ID =>
                            --issue a read for the id field
                            spi_bytes_to_send <= 3 + (pkt_id_field_size_bits/BYTE);
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - (3 + (pkt_id_field_size_bits/BYTE))*BYTE) <=  s0_rx_read_ptr & S0_RX_BUF_READ & x"00";
                           
                            rx_state <= UPDATE_READ_PTR_1;

                        when UPDATE_READ_PTR_1 =>
                            pkt_id_bits <= spi_rx_data(7 downto 0);--read the packet id
                            pkt_id <= pkt_id_bits_to_pkt_type(spi_rx_data(7 downto 0));
                            s0_rx_read_ptr <=std_logic_vector(unsigned(s0_rx_read_ptr) + to_unsigned(1, 16)); --increment read ptr
                        
                            --send updated ptr to the register
                            spi_bytes_to_send <= 5;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 5*BYTE) <=  SnRx_RD_PTR_REG & S0_CONTROL_WRITE & s0_rx_read_ptr;
                            rx_state <= SEND_RECV;
                        when SEND_RECV =>
                            --issue a RECV COMMAND AS PER THE DATASHEET 
                            spi_bytes_to_send <= 4;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 4*BYTE) <=  RECV_COMMAND;
                            rx_state <= READ_DATA;
                        when READ_DATA =>
                            --depending on the id field read a different number of corresponding bytes as each
                            --command will have different byte length depending on id
                            
                            --set the amount of bytes to read based on the pkt_id

                            case (pkt_id) is
                                when HELLO =>
                                    pkt_data_size_bytes := HELLO_pkt_size_bytes; 
                                    spi_bytes_to_send <= 3 + HELLO_pkt_size_bytes;
                                when others =>
                                    pkt_data_size_bytes := 0;
                                    spi_bytes_to_send <= 3;
                            end case;
                                
                            --READ IN the packets data
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 3*BYTE) <=  s0_rx_read_ptr & S0_RX_BUF_READ;
                    
                            rx_state <= UPDATE_READ_PTR_2;
                        
                        when UPDATE_READ_PTR_2 =>
                            --read in recieved data
                            if pkt_data_size_bytes >=1 then
                                rx_pkt_data(pkt_data_size_bytes*BYTE -1 downto 0) <= spi_rx_data(pkt_data_size_bytes*BYTE -1 downto 0);
                            end if;

                            s0_rx_read_ptr <=std_logic_vector(unsigned(s0_rx_read_ptr) + to_unsigned(pkt_data_size_bytes, 16));
                            --send updated ptr to the register
                            spi_bytes_to_send <= 5;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 5*BYTE) <=  SnRx_RD_PTR_REG & S0_CONTROL_WRITE & s0_rx_read_ptr;
                            rx_state <= WRITE_OUT;
                        when WRITE_OUT =>
                            
                            pkt_id_out <= pkt_id; 
                            pkt_data_out <= rx_pkt_data;

                            --issue a RECV COMMAND AS PER THE DATASHEET
                            spi_bytes_to_send <= 4;
                            spi_tx_data_in(spi_tx_reg_MSB downto  spi_tx_reg_width_bits - 4*BYTE) <=  RECV_COMMAND;

                       
                            --Currently per interrupt assertion we only read one packets worth of data
                            --If multiple packets were sent before we processed one we will be one behind in the buffer

                            --if there is still more, to read (bytes_read < size_received) go back to the READ_ID_STATE


                            --ALSO IF BYTES OF GARBAGE WAS RECIEVED(invalid id field) we should move the read pointer forward
                            --SKIPPING THE BYTES RECIEVED AFTER THE INVALID ID FIELD  such that we are back to reading fresh data

                            --if there are no more commands to be read go back to idle
                            state <= IDLE;
        
                    end case;
                end case;          
            end if;
        end if;
    end process;

end architecture;