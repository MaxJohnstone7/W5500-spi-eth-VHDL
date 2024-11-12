--THE NETWORKING CONFIG DESIRED FOR USE IN THE W5500, UP TO THE USER TO CHANGE
--THE CURRENT IMPLEMENTATION TRANSMITS TO ONE PORT ONLY AND RECIEVES FROM ONE PORT ONLY USING UDP
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
package network_config is 

    constant GATEWAY_IP_ADDRESS : std_logic_vector(31 downto 0) := x"C0_A8_00_01";   -- Gateway IP Address = 192.168.0.1
    
    constant SUBNET_MASK_ADDRESS : std_logic_vector(31 downto 0) := x"FF_FF_FF_00";   -- Subnet Mask Address = 255.255.255.0


    constant MAC_ADDRESS : std_logic_vector(47 downto 0) := x"00_08_DC_01_02_03";  -- MAC Address = 00.08.DC.01.02.03

    --HANDLED BY ARP
    -- constant DEST_MAC_ADDRESS : std_logic_vector(47 downto 0) := x"6C_02_E0_5F_60_45";  -- MAC Address = 00.08.DC.01.02.03
    
    constant IP_ADDRESS : std_logic_vector(31 downto 0) := x"a9feecb1";   -- IP Address = 169.254.236.177

    --statically configure on the device the ETH is connected to
    constant DEST_IP_ADDRESS : std_logic_vector(31 downto 0) := x"a9feecf1";   -- Destination IP Address = 169.254.236.241


    constant S0PORT_NUMBER : std_logic_vector(15 downto 0) := x"22B8"; --PORT TO Recieve on = 8888

    constant DEST_S0PORT_NUMBER : std_logic_vector(15 downto 0) := x"1389"; -- PORT TO SEND TO = 5001

end package;