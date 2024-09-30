----------------------------------------------------------------------------------
-- Company:
-- Engineer: Max Johnstone
--
-- Package containing useful types and functions for interfacing with the seven
-- segment displays
----------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package sev_seg_pkg is
	constant NUM_DISPLAYS : integer := 8;
	constant LED_PER_DISPLAY : integer := 7;
    type sev_seg_data_t is array (NUM_DISPLAYS - 1 downto 0) of std_logic_vector(LED_PER_DISPLAY-1 downto 0);
	-- Define character literals type
	type sev_seg_char_t is (C_0, C_1, C_2, C_3, C_4, C_5, C_6, C_7, C_8, C_9, C_A, C_B, C_C, C_D, C_E, C_F, C_H, C_L, C_O, C_BLANK);
    -- Function to map character literals to char_code
    function char_to_leds (c: sev_seg_char_t) return std_logic_vector;
end package sev_seg_pkg;

package body sev_seg_pkg is
	-- Function to map character literals to char_code
	-- Function definition to map character literals to 7-segment display patterns
    function char_to_leds (c: sev_seg_char_t) return std_logic_vector is  -- Adjust range for your display
    begin
        case c is
            when C_0 => return "0000001";  -- 0
            when C_1 => return "1001111";  -- 1
            when C_2 => return "0010010";  -- 2
            when C_3 => return "0000110";  -- 3
            when C_4 => return "1001100";  -- 4
            when C_5 => return "0100100";  -- 5
            when C_6 => return "1100000";  -- 6
            when C_7 => return "0001111";  -- 7
            when C_8 => return "0000000";  -- 8
            when C_9 => return "0001100";  -- 9
            when C_A => return "0001000";  -- A
            when C_B => return "1100000";  -- B
            when C_C => return "0110001";  -- C
            when C_D => return "1000010";  -- D
            when C_E => return "0110000";  -- E
            when C_F => return "0111000";  -- F
            when C_H => return "1001000";  -- H
            when C_L => return "1110001";  -- L
            when C_O => return "0000001";  -- O
            when C_BLANK => return b"1111111"; -- blank display
            when others => return b"1111111";  -- Default case: All segments off
        end case;
    end function char_to_leds;   
end package body sev_seg_pkg;
