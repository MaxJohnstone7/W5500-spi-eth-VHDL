--THIS PACAKGE IS WHERE YOU CAN DEFINE YOUR OWN TYPES FOR USE WITH THE ETHERNET

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
--FIRST 4 bits IDENTIFY THE TYPE OF COMMAND 
--CURRENTLY NO OTHER INFORMATION is transmitted along with the command
--but allowances should probably be made for this in the future
package pkt_types_pkg is
    type pkt_type_t is (HELLO,SEG_DISPLAY_DATA,UNKNOWN);
    constant pkt_id_field_size_bits : integer := 8; 
    constant max_packet_data_bytes : integer := 2; --maximum size of hte pkt_data_field

    --constants for fields of individual packets 
    constant HELLO_pkt_size_bytes : integer := 0; --how many bytes of data after the header
    constant SEG_DISPLAY_DATA_PKT_SIZE_BYTES : integer := 8; 


    function pkt_id_bits_to_pkt_type(bits: std_logic_vector(pkt_id_field_size_bits-1 downto 0)) return pkt_type_t;
    function pkt_id_to_bit(pkt_id: pkt_type_t) return std_logic_vector ;
end package pkt_types_pkg;

package body pkt_types_pkg is

    function pkt_id_bits_to_pkt_type(bits: std_logic_vector(pkt_id_field_size_bits-1 downto 0)) return pkt_type_t is
    begin
        case bits is
            when x"00" => return HELLO;
            when x"01" => return TOGGLE_QUADRATURE;
            when others => return UNKNOWN; --CURRENTLY THE ONLY FUNCTION DEFINED
        end case;

    end function;

    function pkt_id_to_bit(pkt_id: pkt_type_t) return std_logic_vector is 
    begin
        case pkt_id is
            when HELLO => return x"00";
            when TOGGLE_QUADRATURE => return x"01";
            when others => return x"04"; --CURRENTLY THE ONLY FUNCTION DEFINED
        end case;

    end function;

end package body pkt_types_pkg;

