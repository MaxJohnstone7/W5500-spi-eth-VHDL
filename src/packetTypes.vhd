--THIS PACAKGE IS WHERE YOU CAN DEFINE YOUR OWN TYPES FOR USE WITH THE ETHERNET
--ADD YOUR TYPE TO THE PKT_TYPE_T, type and then its corresponding data width to
--the pkt_id_to_data_byte_size function 

--NOTE : Currently data length of a packet type is fixed
--I future could add extra read packet length phase to remove
--uneccary reading overhead.


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;



package pkt_types_pkg is


    type pkt_type_t is (HELLO,SEG_DISPLAY_DATA,UNKNOWN); --add your packet types here

    constant pkt_id_field_size_bytes : integer := 1; 



    constant pkt_id_field_size_bits : integer := pkt_id_field_size_bytes*8;

    constant max_packet_data_bytes : integer := 2; --maximum size of the pkt_data_field

    --constants for fields of individual packets 
    constant HELLO_pkt_size_bytes : integer := 0; --how many bytes of data after the header
    constant SEG_DISPLAY_DATA_PKT_SIZE_BYTES : integer := 8; 


    function pkt_id_bits_to_pkt_type(bits: std_logic_vector(pkt_id_field_size_bits-1 downto 0)) return pkt_type_t;
    function pkt_id_to_bit(pkt_id: pkt_type_t) return std_logic_vector ;
    function pkt_id_to_data_byte_size(pkt_id : pkt_type_t) return natural;
end package pkt_types_pkg;

package body pkt_types_pkg is

    --function defines the size of the data fields for different packet types
    --the user should add their types here
    function pkt_id_to_data_byte_size(pkt_id : pkt_type_t) return natural is 
    begin
        case pkt_id is
            when HELLO => return 0;
            when SEG_DISPLAY_DATA => return 8; --8 CHARS for display, 1 byte each
            when others => return 0; 
        end case;

    end function 

    function pkt_id_bits_to_pkt_type(bits: std_logic_vector(pkt_id_field_size_bits-1 downto 0)) return pkt_type_t is
        variable temp_integer : to_integer(to_unsigned(bits,bits'length));
    begin
        return pkt_type_t'VAL(temp_integer); 
    end function;

    function pkt_id_to_bit(pkt_id: pkt_type_t) return std_logic_vector is 
        variable temp_integer : integer := pkt_id'POS(pkt_type_t) --cast to integer
    begin
        --cast integer to SLV
        return std_logic_vector(to_unsigned(temp_integer, pkt_id_field_size_bits)); 
    end function;

end package body pkt_types_pkg;

