library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity buffered_spi_tb is
end buffered_spi_tb;

architecture behavioural of buffered__spi_tb is

constant spi_tx_reg_width_bytes : integer := 8;
constant spi_tx_reg_width_bits : integer := spi_tx_reg_width_bytes *8;
constant spi_rx_reg_width_bytes : integer := 8;
constant spi_rx_reg_width_bits : integer := spi_rx_reg_width_bytes *8;

signal clk : std_logic := '1'; 
signal reset : std_logic := '1'; 
signal spi_miso : std_logic := '1'; 
signal spi_mosi : std_logic := '1'; 
signal spi_sck : std_logic := '1'; 
signal spi_ss : std_logic := '1'; 

signal spi_tx_data : std_logic_vector(spi_tx_reg_width_bits  -1 downto 0) := (others =>'0');
signal spi_rx_data : std_logic_vector(spi_rx_reg_width_bits -1 downto 0) := (others=>'0');
signal bytes_to_send : integer := 2;
signal spi_enable : std_logic := '0';
signal spi_active : std_logic;


signal data_for_master : std_logic_vector(15 downto 0) := x"AAAA";
signal data_from_master : std_logic_vector(15 downto 0) := x"0000";

signal simdone : boolean := false;

begin

    -- Set the first two bytes to FF
    spi_tx_data(spi_tx_reg_width_bits - 1 downto spi_tx_reg_width_bits  - 16) <= x"AAAA"; -- Set the first two bytes to 55
    spi_miso <= data_for_master(15); --byte to send is the MSB
    


    max_spi_inst : ENTITY work.max_spi
    generic map (CLK_FREQ => 100_000_000,
            SPI_FREQ => 10_000_000,
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
            spi_miso => spi_miso,
            spi_mosi => spi_mosi,
            spi_sck => spi_sck,
            spi_ss => spi_ss,

            enable => spi_enable,

            spi_tx_data_in  => spi_tx_data,
            spi_rx_data_out => spi_rx_data ,

            --determines how many times to shift the spi_tx_vector.
            bytes_to_send => bytes_to_send,

            --wether SPI is currnetly transmittiing
            spi_active => spi_active
        );
    
    clock_process : process begin
        if simdone = false then
            wait for 5 ns;
            clk <= not clk;
        else
            wait;
        end if;
    end process;

    shift_reg_process : process begin       
        if simdone = false then
            --recieve on rising, shift out on falling
            wait until rising_edge(spi_sck);
            --sample in on the rising edge
            data_from_master <= data_from_master(14 downto 0) & '0';
            wait until falling_edge(spi_sck);
            --data to recieve shifts out on falling
            data_for_master <= data_for_master(14 downto 0) & '0';
        else
            wait;
        end if;
    data_from_master(0) <= spi_mosi; --recieve at LSB position
    end process;

    stim_proc : process 
    begin
        wait for 20 ns;
        reset <= '0';
        wait for 103 ns;
        spi_enable <= '1';
        wait until rising_edge(clk);
        spi_enable <= '0';
        wait for 50 us;
  
        

        simdone <= true;
        wait;
    end process;
    

end architecture;