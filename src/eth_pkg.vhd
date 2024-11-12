--THIS PACKAGE EXISTS TO ALLOW MODULES WHICH PROVIDE INPUT TO THE w5500_eth_module to
--cast their data to a valid data_in input vector and acess the debug state types provided
--as outputs by the module.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.pkt_types_pkg.all;

package w5500_pkg is
    type eth_state_t is (IDLE, SETUP, SENDING, RECIEVING);
    type eth_rx_state_t is (GET_SIZE_RECIEVED, CLEAR_INTERRUPT, READ_ID,UPDATE_READ_PTR_1, SEND_RECV, READ_DATA,UPDATE_READ_PTR_2, WRITE_OUT);
    type eth_tx_state_t is (READ_WRITE_PTR, WRITE_BUFF, WRITE_WRITE_PTR, SEND);
    constant MAX_ETH_DATA_SIZE : natural := 100;
    --The ethernet module takes a SLV of MAX_ETH_DATA_SIZE, this function will cast a std_logic_vector
    --of arbitrary length such that it can be fed into the port, has to be a multiple of BYTES in length
    function data_slv_to_valid_eth_input (data_size_bytes : natural; data : std_logic_vector ) return std_logic_vector;
end package;

package body w5500_pkg is

function data_slv_to_valid_eth_input (data_size_bytes : natural; data : std_logic_vector ) return std_logic_vector is
    variable result : std_logic_vector(MAX_ETH_DATA_SIZE-1 downto 0) := (others=> '0');
begin
    result(MAX_ETH_DATA_SIZE-1 downto MAX_ETH_DATA_SIZE-data_size_bytes*BYTE) := data;
    return result;
end function;

end package body;