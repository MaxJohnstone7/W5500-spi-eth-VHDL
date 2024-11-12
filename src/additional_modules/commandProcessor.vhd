
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
USE ieee.numeric_std.all;
use work.pkt_types_pkg.all;
use work.sev_seg_pkg.all;

entity commandProcessor is 
generic (
    clk_freq : integer := 100_000_000
);
port (
    clk : in std_logic;
    command_id : in pkt_type_t;
    command_data : in std_logic_vector(max_packet_data_bytes*BYTE -1 downto 0); --command data to be processed
    sev_seg_data : out sev_seg_data_t := (others => (others => '0'))
);

end commandProcessor;

architecture behavioural of commandProcessor is
    constant pkt_start_index :integer := max_packet_data_bytes*BYTE -1;
    --signal for when a command has been sucesfully execute 
    --ensures that the pkt input will not change untill the command has done
    --its intended purpose
    signal command_done : std_logic;
    --maybe need an actual signal so the output of thhe command can change
    --without chaning the previous output

begin


    process_command : process(clk,command_id)
    begin
        if rising_edge(clk) then
            case command_id is
                when HELLO =>
                    --set the seven seg displays to say hello
                    sev_seg_data <= (others => char_to_leds(C_BLANK));
                    sev_seg_data(7) <= char_to_leds(C_H);
                    sev_seg_data(6) <= char_to_leds(C_E);
                    sev_seg_data(5) <= char_to_leds(C_L);
                    sev_seg_data(4) <= char_to_leds(C_L);
                    sev_seg_data(3) <= char_to_leds(C_O);
                when UNKNOWN =>
                    sev_seg_data <= (others => char_to_leds(C_BLANK));
                    sev_seg_data(7) <= char_to_leds(C_E);
                    sev_seg_data(6) <= char_to_leds(C_E);
                    sev_seg_data(5) <= char_to_leds(C_E);
                    sev_seg_data(4) <= char_to_leds(C_E);
                    sev_seg_data(3) <= char_to_leds(C_E);
                when others =>
                    NULL;
            end case;
        end if;


    end process;


end architecture;