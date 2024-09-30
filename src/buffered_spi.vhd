library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

-- HAS ONE LARGE SHIFT REGISTER which data is latched into when the enable signal is asserted.
-- DATA IS SHIFTED OUT FROM THE MSB OF THE SPI_TX_DATA_IN vector
-- DATA IS SHIFTED INTO THE LSB OF THE SPI_TX_DATA_OUT vector.
-- These shifts ocurr on every clock cycle

-- USE: 
-- Setup the data you want to send 
    -- Setup the spi_tx_data_in variable. e.g. spi_tx_data_in <= 
    -- Set up the bytes to send variable
-- Data & others=> ('0')

-- If spi_bytes_to_send
-- Inside a process check if SPI_DONE has been assereted.
-- clear this signal before a new transaction asserting the ### signal for 1 clock cycle;


-- each spi_sck rising edge the leftmost bit is shifted out onto the mosi line
--
-- once an internal counter has incremented to = spi_bytes_to_send*8


entity buffered_spi is
    generic (CLK_FREQ : integer := 100_000_000;
             SPI_FREQ : integer := 1_000_000;
             CPOL : std_logic := '0';
             -- determines how many byte can be passed at once to the SPI module
             -- for writing
             spi_tx_reg_width_bytes : integer := 64;
             spi_rx_reg_width_bytes : integer := 64
            );
    port(
            clk : in std_logic;
            reset : in std_logic;
            spi_miso : in std_logic;
            spi_mosi : out std_logic := '0';
            spi_sck : out std_logic := '0';
            spi_ss : out std_logic := '1';


            --starts a transaction by reading in spi_tx_data
            enable : in std_logic;

            --data to be shifted left every SPI clock cycle
            spi_tx_data_in : in std_logic_vector(spi_tx_reg_width_bytes*8 - 1 downto 0);
            spi_rx_data_out : out std_logic_vector(spi_rx_reg_width_bytes*8 - 1 downto 0);

            --determines how many times to shift the spi_tx_vector.
            bytes_to_send : in integer range 0 to (spi_tx_reg_width_bytes*BYTE -1);

            --wether SPI is currnetly transmittiing
            spi_active : out std_logic
        );
end buffered_spi;

architecture behavioural of buffered_spi is
    constant spi_tx_reg_width_bits : integer := spi_tx_reg_width_bytes*8;
    constant spi_rx_reg_width_bits : integer := spi_rx_reg_width_bytes*8;
    -- type for the SPI state machine
    type spi_state is (idle_t, start_t, transmit_t);
    signal state : spi_state := idle_t;


    -- counter for the SPI clock
    signal SPI_BIT_PERIOD : integer := CLK_FREQ / SPI_FREQ;
    signal spi_clk_counter : integer := 0;

    
    signal spi_tx_data : std_logic_vector(spi_tx_reg_width_bytes*8 - 1 downto 0) := (others => '0');
    signal spi_rx_data : std_logic_vector(spi_rx_reg_width_bytes*8 - 1 downto 0) := (others => '0');
    signal bits_sent_counter : integer := 0;
 

begin

    spi_state_machine : process(clk,spi_tx_data) is
    begin
        if rising_edge(clk) then
            if reset = '1' then
                state <= idle_t;
            else
            case state is
                when idle_t =>
                    -- when in idle the tx_data is read in
                    spi_rx_data_out <= spi_rx_data;
                    spi_tx_data <= spi_tx_data_in;
                    bits_sent_counter <= 0;
                    spi_ss <= '1';
                    spi_sck <= CPOL;
                    spi_clk_counter <= 0;
                    spi_active <= '0';

                    --Trigger transaction on rising edge of the enable pin.
                    --This ensure a new transaction will only begin if the upper level
                    --Module using SPI has deasserted the enable pin since last transaction
                    if enable = '1' then 
                        state <= start_t;
                        spi_active <= '1';--entering the transmit state, now busy.
                        spi_rx_data <= (others => '0'); --Clear rx buffer before starting
                    end if;
                when start_t =>
                    -- set ss low in preparation for transmission. wait for one bit
                    -- period to ensure it is recognised, and then move to transmission
                    -- state. 
                    spi_ss <= '0';
                    if spi_clk_counter = SPI_BIT_PERIOD - 1 then
                        state <= transmit_t;
                        spi_clk_counter <= 0;
                    else 
                        spi_clk_counter <= spi_clk_counter + 1;
                    end if;

                when transmit_t => 
                    -- if we are done sending
                    if bits_sent_counter = bytes_to_send*8  then
                        state <= idle_t; -- WE ARE DONE GO BACK TO IDLE
                        spi_clk_counter <= 0;
                        
                    --shift data out on falling edge so it is held for half a clock period before sampling
                    elsif spi_clk_counter = SPI_BIT_PERIOD-1 then 
                        spi_sck <= CPOL; --make clock change
                        spi_tx_data <= spi_tx_data(spi_tx_reg_width_bits -2 downto 0) & '0';
                        bits_sent_counter <= bits_sent_counter +1;
                        spi_clk_counter <= 0;  

                    --read data in on the rising edge
                    elsif spi_clk_counter  = SPI_BIT_PERIOD/2 then
                        spi_sck <= not CPOL; --make clock change              
                        spi_rx_data  <= spi_rx_data(spi_rx_reg_width_bits -2 downto 0) & spi_miso;
                        spi_clk_counter <= spi_clk_counter +1;
                    else 
                        spi_clk_counter <= spi_clk_counter +1;
                    end if;


                end case;
            end if;
        end if;
    --MOSI and MISO are always connected to the following
    --bits of the rx and tx registers respectivley
    spi_mosi <= spi_tx_data(spi_tx_reg_width_bits -1);
    end process;

    
end architecture behavioural;