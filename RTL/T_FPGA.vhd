----------------------------------------------------------------------
-- Created by SmartDesign Tue Nov 14 14:10:41 2017
-- Version: v11.8 SP2 11.8.2.4
----------------------------------------------------------------------
 
----------------------------------------------------------------------
-- Libraries
----------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
Library work;
use work.pkg_pwm.all;
use work.RotaryEncoderPckg.all;
use work.pkg_adc.all;
use work.pkg_apb3.all;

library smartfusion2;
use smartfusion2.all;
----------------------------------------------------------------------
-- MC_System_top entity declaration
----------------------------------------------------------------------
entity T_FPGA is
    -- Port list
    port(
        UART_RX    : in   std_logic;
        UART_TX : out   std_logic;
        -- Inputs
        DEVRST_N_0       : in    std_logic;
        BTN_1           : in    std_logic;
        BTN_2           : in    std_logic;
 --ADC 
    SPI_0_DI    : IN     STD_LOGIC;                             --master in, slave out
    SPI_0_DO    : OUT    STD_LOGIC;                             --master out, slave in
    SPI_0_CLK : out std_logic;
    clk_adc : out std_logic;
    reset_adc : out std_logic;
    SPI_0_SS0 : out std_logic;
        -- Outputs
        LED    : out   std_logic_vector(3 downto 0);
    --PMOD
        PMOD_P    : inout   std_logic_vector(3 downto 0);
        PMOD_N    : inout   std_logic_vector(3 downto 0);


	Pulse_In : In   std_logic_vector(3 downto 0) ; -- Width represent disantce
	Trig_out :  out std_logic_vector(3 downto 0) -- >10us sperate by 83.3ms
        -- Inouts
        );
end T_FPGA;

----------------------------------------------------------------------
-- MC_System_top architecture body
----------------------------------------------------------------------
architecture RTL of T_FPGA is
----------------------------------------------------------------------
-- Component declarations
----------------------------------------------------------------------
-- MC_System
component  sysreset
    port (
        DEVRST_N: in    std_logic;
        POWER_ON_RESET_N : out std_logic

    );
end component;
component PLL_Sound_speed is
    -- Port list
    port(
        -- Inputs
        CLK0 : in  std_logic;
        -- Outputs
        GL0      : out std_logic;
        LOCK     : out std_logic
        );
end component;
component PMOD_MOTOR is
port (
    --<port_name> : <direction> <type>;
	PMOD1 : inout std_logic_vector(3 downto 0)  ; -- example
	PMOD2 : inout std_logic_vector(3 downto 0)  ; -- example
    Motor_EN1 :IN std_logic;
    Motor_EN2 : IN std_logic;
    Motor_DIR2: IN std_logic;
    Motor_DIR1 :IN std_logic;
    
    ENC1 : OUT std_logic_vector(1 downto 0);  -- example
    ENC2 : OUT std_logic_vector(1 downto 0)  -- example
    --<other_ports>;
);
end component;
----------------------------------------------------------------------
-- Signal declarations
----------------------------------------------------------------------
signal A0_ADC_CS_O_0                    : std_logic;
signal A0_ADC_SCK_O_0                   : std_logic;
signal A0_ADC_SDO_O_0                   : std_logic;
signal A0_PWM_UH_O_0                    : std_logic;
signal A0_PWM_UL_O_0                    : std_logic;
signal A0_PWM_VH_O_0                    : std_logic;
signal A0_PWM_WH_O_0                    : std_logic;
signal A0_PWM_WL_O_0                    : std_logic;
signal A0_RUNNING_LED_1                 : std_logic;
signal GPIO_0_M2F_net_0                 : std_logic;
signal MC_System_0_AMBA_SLAVE_0_PADDR   : std_logic_vector(31 downto 0);
signal MC_System_0_AMBA_SLAVE_0_PENABLE : std_logic;
signal MC_System_0_AMBA_SLAVE_0_PRDATA  : std_logic_vector(31 downto 0);
signal MC_System_0_AMBA_SLAVE_0_PREADY  : std_logic;
signal MC_System_0_AMBA_SLAVE_0_PSELx   : std_logic;
signal MC_System_0_AMBA_SLAVE_0_PSLVERR : std_logic;
signal MC_System_0_AMBA_SLAVE_0_PWDATA  : std_logic_vector(31 downto 0);
signal MC_System_0_AMBA_SLAVE_0_PWRITE  : std_logic;
signal MC_System_0_FIC_0_CLK            : std_logic;
signal MC_System_0_FIC_0_LOCK           : std_logic;
signal USB_ULPI_STP_1                   : std_logic;
signal GPIO_0_M2F_net_1                 : std_logic;
signal A0_PWM_UL_O_0_net_0              : std_logic;
signal A0_PWM_UH_O_0_net_0              : std_logic;
signal A0_PWM_WH_O_0_net_0              : std_logic;
signal A0_PWM_WL_O_0_net_0              : std_logic;
signal A0_ADC_SCK_O_0_net_0             : std_logic;
signal A0_RUNNING_LED_1_net_0           : std_logic;
signal A0_PWM_VH_O_0_net_0              : std_logic;
signal A0_ADC_CS_O_0_net_0              : std_logic;
signal A0_ADC_SDO_O_0_net_0             : std_logic;
signal A0_RUNNING_LED_1_net_1           : std_logic;
signal USB_ULPI_STP_1_net_0             : std_logic;
----------------------------------------------------------------------
-- TiedOff Signals
----------------------------------------------------------------------
signal AMBA_SLAVE_1_PRDATAS1_const_net_0: std_logic_vector(31 downto 0);
signal PWM_OUT_20: std_logic_vector(19 downto 0);
signal a_i,b_i : std_logic_vector(1 downto 0);
signal VCC_net                          : std_logic;
signal GND_net                          : std_logic;
signal DEVRST_N                            : std_logic;
signal        apb3_PWM :  apb3;
signal        apb3_PWM_Back :  apb3_Back;
signal        apb3_ADC :  apb3;
signal        apb3_ADC_Back :  apb3_Back;
signal        apb3_Encoder :  apb3;
signal        apb3_Encoder_Back :  apb3_Back;
signal        apb3_SR04 :  apb3;
signal        apb3_SR04_Back :  apb3_Back;
signal apb3_slave_array :apb3_array(0 to 3);
signal apb3_slave_array_Back :apb3_array_Back(0 to 3);
signal clk_sound_speed :     std_logic;



begin
----------------------------------------------------------------------
-- Constant assignments
----------------------------------------------------------------------
 AMBA_SLAVE_1_PRDATAS1_const_net_0 <= B"00000000000000000000000000000000";
 VCC_net                           <= '1';
 GND_net                           <= '0';
----------------------------------------------------------------------
-- Top level output port assignments
----------------------------------------------------------------------

----------------------------------------------------------------------
-- Component instances
----------------------------------------------------------------------
-- BLDC_Encoder_Axis_0
reset_adc<=DEVRST_N;
-- MC_System_0
sys_reset_0 : sysreset
    port map(
        DEVRST_N=>DEVRST_N_0,
        POWER_ON_RESET_N=>DEVRST_N
    );
mss_Robot_sb_0 : entity work.mss_top_sb
    port map( 
        -- Inputs
        fab_reset_n               => DEVRST_N,

        DEVRST_N               => DEVRST_N,
        AMBA_SLAVE_0_PREADYS0  => MC_System_0_AMBA_SLAVE_0_PREADY,
        AMBA_SLAVE_0_PSLVERRS0 => MC_System_0_AMBA_SLAVE_0_PSLVERR,
        AMBA_SLAVE_0_PRDATAS0  => MC_System_0_AMBA_SLAVE_0_PRDATA,
        -- Outputs
        FIC_0_CLK              => MC_System_0_FIC_0_CLK,
FAB_CCC_GL1 => clk_adc,
        AMBA_SLAVE_0_PSELS0    => MC_System_0_AMBA_SLAVE_0_PSELx,
        AMBA_SLAVE_0_PENABLES  => MC_System_0_AMBA_SLAVE_0_PENABLE,
        AMBA_SLAVE_0_PWRITES   => MC_System_0_AMBA_SLAVE_0_PWRITE,
        FIC_0_LOCK             => MC_System_0_FIC_0_LOCK,

        AMBA_SLAVE_0_PADDRS    => MC_System_0_AMBA_SLAVE_0_PADDR,
        AMBA_SLAVE_0_PWDATAS   => MC_System_0_AMBA_SLAVE_0_PWDATA,
        Power_on_reset_n       =>open,
        mss_ready              =>open,

        MMUART_0_TXD_M2F       =>UART_TX,
        MMUART_0_RXD_F2M       =>UART_RX,
        SPI_0_DO_M2F                =>open  ,
        -- Inouts-- Inouts,
        SPI_0_CLK_M2F               =>open ,
        SPI_0_CLK_F2M               =>'Z' ,
        SPI_0_DI_F2M                =>SPI_0_DI  ,
        SPI_0_SS0_M2F               =>open, 
        SPI_0_SS0_M2F_OE               =>open, 
        SPI_0_SS0_F2M               =>'Z' 
        );

splitter_apb3_assync : entity work.splitter_apb3_assync 
generic map( 
        NUMBER_SLAVE => 4,
        START_ADDRESS =>(x"0000",x"0100",x"0200",x"0300"),
        END_ADDRESS =>  (x"00A0",x"01FF",x"02FF",x"03FF")
)
 port map(
      apb3_master.clk=> MC_System_0_FIC_0_CLK,
      apb3_master.sel=> MC_System_0_AMBA_SLAVE_0_PSELx,
      apb3_master.nRW=> MC_System_0_AMBA_SLAVE_0_PWRITE,
      apb3_master.enable=> MC_System_0_AMBA_SLAVE_0_PENABLE,
      apb3_master.address=> MC_System_0_AMBA_SLAVE_0_PADDR,
      apb3_master.wdata=> MC_System_0_AMBA_SLAVE_0_PWDATA,


      apb3_master_Back.rdata=> MC_System_0_AMBA_SLAVE_0_PRDATA,
      apb3_master_Back.ready=> MC_System_0_AMBA_SLAVE_0_PREADY,
      apb3_master_Back.slverr=> MC_System_0_AMBA_SLAVE_0_PSLVERR,

      apb3_slave_array=>apb3_slave_array,
      apb3_slave_array_Back=>apb3_slave_array_Back

     );
apb3_PWM <=apb3_slave_array(0)  ;
apb3_slave_array_Back(0)<=apb3_PWM_Back   ;

apb3_Encoder <=apb3_slave_array(1)  ;
apb3_slave_array_Back(1) <= apb3_Encoder_Back  ;
 apb3_ADC <=apb3_slave_array(2);
apb3_slave_array_Back(2)<=        apb3_ADC_Back   ;

apb3_SR04 <=apb3_slave_array(3)  ;
apb3_slave_array_Back(3)<=       apb3_SR04_Back   ;
 
    multiple_pwm_20_0 : multiple_pwm_20
        -- port map
        port map( 
            -- Inputs
            clk => MC_System_0_FIC_0_CLK,
            
            reset => DEVRST_N,

            apb3_master =>apb3_PWM,
            apb3_master_Back =>apb3_PWM_Back, -- Outputs
            PWM_OUT_20 => PWM_OUT_20

            -- Inouts

        );

PMOD_motor_u : PMOD_MOTOR 
port map(
    --<port_name> : <direction> <type>;
	PMOD1 =>PMOD_P,
    PMOD2 =>PMOD_N,
    Motor_EN1 =>PWM_OUT_20(0),
    Motor_EN2 =>PWM_OUT_20(2),
    Motor_DIR2=>PWM_OUT_20(1),
    Motor_DIR1=>PWM_OUT_20(3),
    
    ENC1(0) =>a_i(0),  -- example
    ENC1(1) =>b_i(0),  -- example
    ENC2(0) =>a_i(1),  -- example
    ENC2(1) =>b_i(1)  -- example
    --<other_ports>;
);


SPI_0_CLK<=clk_adc;
u_adc_channel : adc_channel_reader 
  PORT map(
    clk_spi =>SPI_0_CLK,
    reset_n =>DEVRST_N,                             --asynchronous reset
    miso   =>SPI_0_DI,                             --master in, slave out
    mosi   =>SPI_0_DO,                      --master out, slave in
    cs_spi =>SPI_0_SS0,
    enable=>'1',
            apb3_master =>apb3_adc,
            apb3_master_Back =>apb3_adc_Back -- Outputs
);

u_Multiple_SR04_04 :  entity work.Multiple_SR04_04
  PORT map(
    reset =>DEVRST_N,                             --asynchronous reset
    Trig_out   =>Trig_out,                             --master in, slave out
    Pulse_In   =>Pulse_In,                      --master out, slave in
    clk_sound_speed =>clk_sound_speed,
            apb3_master =>apb3_sr04,
            apb3_master_Back =>apb3_sr04_Back -- Outputs
);


Multiple_RotaryEncoderWithCounter_2_u : entity work.Multiple_RotaryEncoderWithCounter_2 
  PORT map(
    reset =>DEVRST_N,                             --asynchronous reset
    a_i   =>a_i,                             --master in, slave out
    b_i   =>b_i,                      --master out, slave in
    --enable=>'1',
            apb3_master =>apb3_Encoder,
            apb3_master_Back =>apb3_Encoder_Back -- Outputs
);
PLL_Sound_speed0 : PLL_Sound_speed 
    -- Port list
    port map(
        -- Inputs
        CLK0 =>MC_System_0_FIC_0_CLK,
        -- Outputs
        GL0      =>clk_sound_speed,
        LOCK     =>open
        );

LED<=PWM_OUT_20(3 downto 0);


end RTL;
