library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use work.Config.all;
use work.pkt_types_pkg.all;
use work.sev_seg_pkg.all;
use work.eth_state_pkg.all;

entity w5500_spi_ethernet_tb is
end w5500_spi_ethernet_tb;

architecture behavioural of w5500_spi_ethernet_tb is

--spi outputs
signal MISO_ETH : std_logic := '0'; --will correspond to a HELLO PACKET
signal mosi_eth : std_logic := '0';
signal cs_eth : std_logic := '0';
signal sclk_eth : std_logic := '0';

--ETH SIGNALS
signal CLK : std_logic := '0';
signal ack_data_read :std_logic := '0';
signal eth_pkt_id : pkt_type_t := UNKNOWN;
signal eth_pkt_data : std_logic_vector(max_packet_data_bytes*BYTE -1 downto 0) := (others=> '0');
signal write_ptr : std_logic_vector(15 downto 0);
signal int : std_logic := '0';
signal adc_sample_ready : std_logic := '0';

--internal eth signals
signal state : eth_state_t;
signal rx_state : eth_rx_state_t;
signal tx_state : eth_tx_state_t;



signal seq_num_ascii : std_logic_vector(7 downto 0) := x"01";
signal adc_vals_as_hex : std_logic_vector(4*BYTE*NUM_ADC -1 downto 0) := x"33304233333333333330423333333333";

--
signal simdone : boolean := false;
--SEVEN SIG SIGNAL
signal sev_seg_data : sev_seg_data_t := (others => (others => '0'));



constant spi_speed : integer := 10_000_000; --20MHZ
constant spi_bit_period :time := 100 ns; --clock period
constant tx_packet_size : integer := 4*NUM_ADC + 1;

begin

--WIZNET INSTANCE
    WIZNET_INTERFACE_INST : entity work.max_ethernet
    generic map (
        clk_freq => 100_000_000,
        spi_freq => spi_speed, --just for simulation
        packet_size_bytes => tx_packet_size --sequence number takes one byte,each adc one ascii char
    )
    Port map (
        clk => CLK,
        reset => '0',
        miso => MISO_ETH,
        mosi => MOSI_ETH,
        ss => CS_ETH,
        sck => SCLK_ETH,
        new_data_avail => adc_sample_ready,
        ack_new_data => ack_data_read,
        data_in => adc_vals_as_hex & seq_num_ascii,
        Int => INT,
        pkt_id_out=> eth_pkt_id,
        pkt_data_out=> eth_pkt_data,
        --debug info
        write_ptr => write_ptr,
        state_d => state,
        rx_state_d => rx_state,
        tx_state_d => tx_state
    );


    clock_process : process begin
        if simdone = false then
            wait for 5 ns; -- 100MHZ
            clk <= not clk;
        else
            wait;
        end if;
    end process;

    stim_proc : process 
    begin

        wait for 100 us; --wait for module init delay
        wait until state = IDLE for 100 us;
        int <= '0'; --simulate rx reception of hello
        wait until state = RECIEVING for 50 us;
        int <= '1'; --deassert interrupt
        wait until state = IDLE for 50 us; 
        --assert that the packet id has been set to hello from reception
        assert (eth_pkt_id = HELLO) report "Hello not sucesfully received" severity ERROR;

        adc_sample_ready <= '1'; -- simulate a transmisison
        wait until state = SENDING for 50 us;
        adc_sample_ready <= '0';
        wait until state = IDLE for 50 us;
        assert (write_ptr= std_logic_vector(to_unsigned(tx_packet_size,16))) report "Write ptr not correctly incremented" severity ERROR;
        

        --tests adc data read correctly
        
        

        simdone <= true;
        wait;
    end process;

    


end architecture;