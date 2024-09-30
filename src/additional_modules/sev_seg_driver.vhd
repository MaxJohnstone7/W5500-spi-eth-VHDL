----------------------------------------------------------------------------------
-- Company:
-- Engineer: Max Johnstone
--
-- Create Date: 15.04.2023 00:18:31
-- Design Name:
-- Module Name: sev_seg_driver - Behavioral
-- Project Name:
-- Target Devices:
-- Tool Versions:
-- Description:
--
-- Dependencies:
--
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
----------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use work.sev_seg_pkg.all;


ENTITY sev_seg_driver IS
	GENERIC (
		clk_freq : integer := 100_000_000;
		refresh_rate : integer := 1000
	);
	PORT (
		clk : in std_logic;
		leds_in  : IN sev_seg_data_t;
		disp_out : out STD_LOGIC_VECTOR(NUM_DISPLAYS-1 DOWNTO 0) := x"FF";
		leds_out : out std_logic_vector(LED_PER_DISPLAY-1 downto 0) := "1111111"
	);
END sev_seg_driver;

ARCHITECTURE Behavioral OF sev_seg_driver IS

    constant display_period_ticks : integer := CLK_FREQ / refresh_rate;
	signal disp_index :integer := 0;
	signal display_clk_counter : integer := 0;
BEGIN
	refresh_display : process (clk)
	begin
	    if rising_edge(clk) then
            if display_clk_counter  = display_period_ticks - 1 then
                display_clk_counter <= 0;
                --shift to next display
                disp_index <= disp_index + 1;
                if disp_index = NUM_DISPLAYS-1 then
                    disp_index <= 0;
                end if;
            else 
                display_clk_counter <= display_clk_counter  + 1;
            end if;	
            leds_out <= leds_in(disp_index);
            disp_out <= (others => '1');
            disp_out(disp_index) <= '0';
        end if;
	END PROCESS;
END Behavioral;