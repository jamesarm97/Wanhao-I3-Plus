#include "temperature.h"
#include "ultralcd.h"

#include "Marlin.h"
#include "language.h"
#include "cardreader.h"
#include "temperature.h"
#include "stepper.h"
#include "ConfigurationStore.h"

char check7=0x07;
char check6=0x06;
char check5=0x05;
char check4=0x04;
char mydegbed[4];

static uint8_t tmp_extruder12;
unsigned long oldtime=0;
unsigned long oldtime11=0;
String BT_DATA = "";  
char vs_lcd_data[36];
uint8_t tmp1_extruder;
char sdfilename[20];
uint16_t file_num=0;
float tmp_extru;
char conv[8];
char conv2[1];
static char vs_temp_data;
int tmp_extru_tm;
//int target_temperature_bed_tm
#ifdef ULTRA_LCD
int8_t encoderDiff; /* encoderDiff is updated from interrupt context and added to encoderPosition every LCD update */

/* Configuration settings */
int plaPreheatHotendTemp;
int plaPreheatHPBTemp;
int plaPreheatFanSpeed;

int absPreheatHotendTemp;
int absPreheatHPBTemp;
int absPreheatFanSpeed;


#ifdef ULTIPANEL
static float manual_feedrate[] = MANUAL_FEEDRATE;
#endif // ULTIPANEL

/* !Configuration settings */

//Function pointer to menu functions.
typedef void (*menuFunc_t)();

uint8_t lcd_status_message_level;
char lcd_status_message[LCD_WIDTH+1] = WELCOME_MSG;

#ifdef DOGLCD
#include "dogm_lcd_implementation.h"
#else
#include "ultralcd_implementation_hitachi_HD44780.h"
#endif

/** forward declerations **/

void copy_and_scalePID_i();
void copy_and_scalePID_d();

/* Different menus */
static void lcd_status_screen();
#ifdef ULTIPANEL
extern bool powersupply;
static void lcd_main_menu();
static void lcd_tune_menu();
static void lcd_prepare_menu();
static void lcd_move_menu();
static void lcd_control_menu();
static void lcd_control_temperature_menu();
static void lcd_control_temperature_preheat_pla_settings_menu();
static void lcd_control_temperature_preheat_abs_settings_menu();
static void lcd_control_motion_menu();
#ifdef DOGLCD
#if !defined(MINIPANEL)
static void lcd_set_contrast();
#endif
#endif
static void lcd_control_retract_menu();
static void lcd_sdcard_menu();

static void lcd_quick_feedback();//Cause an LCD refresh, and give the user visual or audiable feedback that something has happend

/* Different types of actions that can be used in menuitems. */
static void menu_action_back(menuFunc_t data);
static void menu_action_submenu(menuFunc_t data);
static void menu_action_gcode(const char* pgcode);
static void menu_action_function(menuFunc_t data);
static void menu_action_sdfile(const char* filename, char* longFilename);
static void menu_action_sddirectory(const char* filename, char* longFilename);
static void menu_action_setting_edit_bool(const char* pstr, bool* ptr);
static void menu_action_setting_edit_int3(const char* pstr, int* ptr, int minValue, int maxValue);
static void menu_action_setting_edit_float3(const char* pstr, float* ptr, float minValue, float maxValue);
static void menu_action_setting_edit_float32(const char* pstr, float* ptr, float minValue, float maxValue);
static void menu_action_setting_edit_float5(const char* pstr, float* ptr, float minValue, float maxValue);
static void menu_action_setting_edit_float51(const char* pstr, float* ptr, float minValue, float maxValue);
static void menu_action_setting_edit_float52(const char* pstr, float* ptr, float minValue, float maxValue);
static void menu_action_setting_edit_long5(const char* pstr, unsigned long* ptr, unsigned long minValue, unsigned long maxValue);
static void menu_action_setting_edit_callback_bool(const char* pstr, bool* ptr, menuFunc_t callbackFunc);
static void menu_action_setting_edit_callback_int3(const char* pstr, int* ptr, int minValue, int maxValue, menuFunc_t callbackFunc);
static void menu_action_setting_edit_callback_float3(const char* pstr, float* ptr, float minValue, float maxValue, menuFunc_t callbackFunc);
static void menu_action_setting_edit_callback_float32(const char* pstr, float* ptr, float minValue, float maxValue, menuFunc_t callbackFunc);
static void menu_action_setting_edit_callback_float5(const char* pstr, float* ptr, float minValue, float maxValue, menuFunc_t callbackFunc);
static void menu_action_setting_edit_callback_float51(const char* pstr, float* ptr, float minValue, float maxValue, menuFunc_t callbackFunc);
static void menu_action_setting_edit_callback_float52(const char* pstr, float* ptr, float minValue, float maxValue, menuFunc_t callbackFunc);
static void menu_action_setting_edit_callback_long5(const char* pstr, unsigned long* ptr, unsigned long minValue, unsigned long maxValue, menuFunc_t callbackFunc);

#define ENCODER_FEEDRATE_DEADZONE 10

#if !defined(LCD_I2C_VIKI)
  #ifndef ENCODER_STEPS_PER_MENU_ITEM
    #define ENCODER_STEPS_PER_MENU_ITEM 5
  #endif
  #ifndef ENCODER_PULSES_PER_STEP
    #define ENCODER_PULSES_PER_STEP 1
  #endif
#else
  #ifndef ENCODER_STEPS_PER_MENU_ITEM
    #define ENCODER_STEPS_PER_MENU_ITEM 2 // VIKI LCD rotary encoder uses a different number of steps per rotation
  #endif
  #ifndef ENCODER_PULSES_PER_STEP
    #define ENCODER_PULSES_PER_STEP 1
  #endif
#endif


/* Helper macros for menus */
#define START_MENU() do { \
    if (encoderPosition > 0x8000) encoderPosition = 0; \
    if (encoderPosition / ENCODER_STEPS_PER_MENU_ITEM < currentMenuViewOffset) currentMenuViewOffset = encoderPosition / ENCODER_STEPS_PER_MENU_ITEM;\
    uint8_t _lineNr = currentMenuViewOffset, _menuItemNr; \
    bool wasClicked = LCD_CLICKED;\
    for(uint8_t _drawLineNr = 0; _drawLineNr < LCD_HEIGHT; _drawLineNr++, _lineNr++) { \
        _menuItemNr = 0;
#define MENU_ITEM(type, label, args...) do { \
    if (_menuItemNr == _lineNr) { \
        if (lcdDrawUpdate) { \
            const char* _label_pstr = PSTR(label); \
            if ((encoderPosition / ENCODER_STEPS_PER_MENU_ITEM) == _menuItemNr) { \
                lcd_implementation_drawmenu_ ## type ## _selected (_drawLineNr, _label_pstr , ## args ); \
            }else{\
                lcd_implementation_drawmenu_ ## type (_drawLineNr, _label_pstr , ## args ); \
            }\
        }\
        if (wasClicked && (encoderPosition / ENCODER_STEPS_PER_MENU_ITEM) == _menuItemNr) {\
            lcd_quick_feedback(); \
            menu_action_ ## type ( args ); \
            return;\
        }\
    }\
    _menuItemNr++;\
} while(0)
#define MENU_ITEM_DUMMY() do { _menuItemNr++; } while(0)
#define MENU_ITEM_EDIT(type, label, args...) MENU_ITEM(setting_edit_ ## type, label, PSTR(label) , ## args )
#define MENU_ITEM_EDIT_CALLBACK(type, label, args...) MENU_ITEM(setting_edit_callback_ ## type, label, PSTR(label) , ## args )
#define END_MENU() \
    if (encoderPosition / ENCODER_STEPS_PER_MENU_ITEM >= _menuItemNr) encoderPosition = _menuItemNr * ENCODER_STEPS_PER_MENU_ITEM - 1; \
    if ((uint8_t)(encoderPosition / ENCODER_STEPS_PER_MENU_ITEM) >= currentMenuViewOffset + LCD_HEIGHT) { currentMenuViewOffset = (encoderPosition / ENCODER_STEPS_PER_MENU_ITEM) - LCD_HEIGHT + 1; lcdDrawUpdate = 1; _lineNr = currentMenuViewOffset - 1; _drawLineNr = -1; } \
    } } while(0)

/** Used variables to keep track of the menu */
#ifndef REPRAPWORLD_KEYPAD
volatile uint8_t buttons;//Contains the bits of the currently pressed buttons.
#else
volatile uint8_t buttons_reprapworld_keypad; // to store the reprapworld_keypad shiftregister values
#endif
#ifdef LCD_HAS_SLOW_BUTTONS
volatile uint8_t slow_buttons;//Contains the bits of the currently pressed buttons.
#endif
uint8_t currentMenuViewOffset;              /* scroll offset in the current menu */
uint32_t blocking_enc;
uint8_t lastEncoderBits;
uint32_t encoderPosition;
#if (SDCARDDETECT > 0)
bool lcd_oldcardstatus;
#endif
#endif//ULTIPANEL

menuFunc_t currentMenu = lcd_status_screen; /* function pointer to the currently active menu */
uint32_t lcd_next_update_millis;
uint8_t lcd_status_update_delay;
uint8_t lcdDrawUpdate = 2;                  /* Set to none-zero when the LCD needs to draw, decreased after every draw. Set to 2 in LCD routines so the LCD gets atleast 1 full redraw (first redraw is partial) */

//prevMenu and prevEncoderPosition are used to store the previous menu location when editing settings.
menuFunc_t prevMenu = NULL;
uint16_t prevEncoderPosition;
//Variables used when editing values.
const char* editLabel;
void* editValue;
int32_t minEditValue, maxEditValue;
menuFunc_t callbackFunc;

// placeholders for Ki and Kd edits
float raw_Ki, raw_Kd;
#endif
static float manual_feedrate[] = {50*60, 50*60, 4*60, 60};

#if (SDCARDDETECT > 0)
bool lcd_oldcardstatus;
#endif

char *itostr2(const uint8_t &x)
{
  int xx=x;
  conv[0]=(xx/10)%10+'0';
  conv[1]=(xx)%10+'0';
  conv[2]=0;
  return conv;

}

void lcd_vs_temp_data()
{
	
	vs_lcd_data[0]=0x5A;
	vs_lcd_data[1]=0xA5;
	vs_lcd_data[2]=0x21;
	vs_lcd_data[3]=0x82;
	vs_lcd_data[4]=0x00;
	vs_lcd_data[5]=0x01;
	
	tmp_extru=degHotend(tmp1_extruder);
	if(tmp_extru>255)
	{
		vs_lcd_data[6]=0x01;
		tmp_extru=tmp_extru-256;
	}
	else
	{	
		vs_lcd_data[6]=0x00;
	}
	vs_lcd_data[7]=(char)((int)tmp_extru);
	vs_lcd_data[8]=0x00;
	vs_lcd_data[9]=0x00;
	vs_lcd_data[10]=0x00;
	vs_lcd_data[11]=(char)((int)degBed());
	vs_lcd_data[12]=0x00;
	vs_lcd_data[13]=fanSpeed*100/255;
	vs_lcd_data[14]=0x00;
	vs_lcd_data[15]=card.percentDone();	
	vs_lcd_data[16]=0x00;
	vs_lcd_data[17]=0x00;
	vs_lcd_data[18]=0x00;
	vs_lcd_data[19]=0x00;
	vs_lcd_data[20]=0x00;
	vs_lcd_data[21]=0x00;
	vs_lcd_data[22]=0x00;
	vs_lcd_data[23]=0x00;
	vs_lcd_data[24]=0x00;
	vs_lcd_data[25]=0x00;

		int tmp_extru;
		tmp_extru=target_temperature[0];
		if(tmp_extru>255)
		{
			vs_lcd_data[26]=0x01;
			tmp_extru=tmp_extru-256;
		}
		else
		{	
			vs_lcd_data[26]=0x00;
		}
		vs_lcd_data[27]=(char)tmp_extru;

	vs_lcd_data[28]=0x00;
	vs_lcd_data[29]=0x00;

	vs_lcd_data[30]=0x00;
	vs_lcd_data[31]=(char)target_temperature_bed;

		int temp_print_percent=feedmultiply;
		if(temp_print_percent>255)
		{
			vs_lcd_data[32]=0x01;
			temp_print_percent=temp_print_percent-256;
		}
		else
		{
			vs_lcd_data[32]=0x00;
		}

		vs_lcd_data[33]=(char)temp_print_percent;

	vs_lcd_data[34]=0x00;
	vs_lcd_data[35]=0x00;


	for(int i=0 ; i<36 ; i++)
	{
		MYSERIAL2.write(vs_lcd_data[i]);
		_delay_ms(2);
	}

	if(IS_SD_PRINTING && card.percentDone()>98)
	{
		if(acceleration>1000)
			lcd_vs_App_Page(0x45);
		else
			lcd_vs_App_Page(0x9D);
	}

}

char *itochr2(const uint8_t &x)
{
  int xx=x;
	conv[0]=(xx/10)%10;
	conv[1]=(xx)%10;
	conv[2]=0;

  conv2[0]=conv[0]*16+conv[1];
  return conv2;
}

#ifdef ULTRA_LCD
/* Main status screen. It's up to the implementation specific part to show what is needed. As this is very display dependend */
static void lcd_status_screen()
{
    if (lcd_status_update_delay)
        lcd_status_update_delay--;
    else
        lcdDrawUpdate = 1;
    if (lcdDrawUpdate)
    {
        lcd_implementation_status_screen();
        lcd_status_update_delay = 10;   /* redraw the main screen every second. This is easier then trying keep track of all things that change on the screen */
    }
#ifdef ULTIPANEL
    if (LCD_CLICKED)
    {
        currentMenu = lcd_main_menu;
        encoderPosition = 0;
        lcd_quick_feedback();
    }

    // Dead zone at 100% feedrate
    if ((feedmultiply < 100 && (feedmultiply + int(encoderPosition)) > 100) ||
            (feedmultiply > 100 && (feedmultiply + int(encoderPosition)) < 100))
    {
        encoderPosition = 0;
        feedmultiply = 100;
    }

    if (feedmultiply == 100 && int(encoderPosition) > ENCODER_FEEDRATE_DEADZONE)
    {
        feedmultiply += int(encoderPosition) - ENCODER_FEEDRATE_DEADZONE;
        encoderPosition = 0;
    }
    else if (feedmultiply == 100 && int(encoderPosition) < -ENCODER_FEEDRATE_DEADZONE)
    {
        feedmultiply += int(encoderPosition) + ENCODER_FEEDRATE_DEADZONE;
        encoderPosition = 0;
    }
    else if (feedmultiply != 100)
    {
        feedmultiply += int(encoderPosition);
        encoderPosition = 0;
    }
	//打印速度比例10-300
    if (feedmultiply < 10)
        feedmultiply = 10;
    if (feedmultiply > 299)
        feedmultiply = 300;
#endif//ULTIPANEL
}

#ifdef ULTIPANEL
static void lcd_return_to_status()
{
    encoderPosition = 0;
    currentMenu = lcd_status_screen;
}

static void lcd_sdcard_pause()
{
    card.pauseSDPrint();
}
static void lcd_sdcard_resume()
{
    card.startFileprint();
}
//extern bool Stopped;
static void lcd_sdcard_rs()
{
//   card.startFileprint();
//	Stopped=false;
	enquecommand_P(PSTR("M1"));
//	enquecommand_P(PSTR("G4"));
}

static void lcd_sdcard_stop()
{
    card.sdprinting = false;
    card.closefile();
    quickStop();
    if(SD_FINISHED_STEPPERRELEASE)
    {
        enquecommand_P(PSTR(SD_FINISHED_RELEASECOMMAND));
    }
    autotempShutdown();
}

/* Menu implementation */
static void lcd_main_menu()
{
    START_MENU();
    MENU_ITEM(back, MSG_WATCH, lcd_status_screen);
    if (movesplanned() || IS_SD_PRINTING)
    {
        MENU_ITEM(submenu, MSG_TUNE, lcd_tune_menu);
    }else{
        MENU_ITEM(submenu, MSG_PREPARE, lcd_prepare_menu);
       // u8g.setFont(u8g_font_6x10_marlin);
    }
    MENU_ITEM(submenu, MSG_CONTROL, lcd_control_menu);
#ifdef SDSUPPORT
    if (card.cardOK)
    {
        if (card.isFileOpen())
        {
            if (card.sdprinting)
                MENU_ITEM(function, MSG_PAUSE_PRINT, lcd_sdcard_pause);
            else
                MENU_ITEM(function, MSG_RESUME_PRINT, lcd_sdcard_resume);
            MENU_ITEM(function, MSG_STOP_PRINT, lcd_sdcard_stop);
			//RS 打印
		//	MENU_ITEM(function, MSG_RESUME_RS, lcd_sdcard_rs);
        }else{
            MENU_ITEM(submenu, MSG_CARD_MENU, lcd_sdcard_menu);
			
			
#if SDCARDDETECT < 1
            MENU_ITEM(gcode, MSG_CNG_SDCARD, PSTR("M21"));  // SD-card changed by user
#endif
        }
    }else{
        MENU_ITEM(submenu, MSG_NO_CARD, lcd_sdcard_menu);
#if SDCARDDETECT < 1
        MENU_ITEM(gcode, MSG_INIT_SDCARD, PSTR("M21")); // Manually initialize the SD-card via user interface
#endif
    }
#endif
    END_MENU();
}

#ifdef SDSUPPORT
static void lcd_autostart_sd()
{
    card.lastnr=0;
    card.setroot();
    card.checkautostart(true);
}
#endif

void lcd_preheat_pla()
{
    setTargetHotend0(plaPreheatHotendTemp);
    setTargetHotend1(plaPreheatHotendTemp);
    setTargetHotend2(plaPreheatHotendTemp);
    setTargetBed(plaPreheatHPBTemp);
    fanSpeed = plaPreheatFanSpeed;
    lcd_return_to_status();
    setWatch(); // heater sanity check timer
}

void lcd_preheat_abs()
{
    setTargetHotend0(absPreheatHotendTemp);
    setTargetHotend1(absPreheatHotendTemp);
    setTargetHotend2(absPreheatHotendTemp);
    setTargetBed(absPreheatHPBTemp);
    fanSpeed = absPreheatFanSpeed;
    lcd_return_to_status();
    setWatch(); // heater sanity check timer
}
static void lcd_autohome()
{
	//如果Z轴小于10毫米,就往上升10毫米,再归零.
	if(current_position[Z_AXIS]<10)
	{
	//	destination[Z_AXIS]=
		current_position[Z_AXIS]+=10;
		plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] , current_position[E_AXIS], manual_feedrate[X_AXIS]/60, active_extruder);
	}
	
	//归原点
	enquecommand_P((PSTR("G28"))); // move all axis home
//	PSTR("G28");
	//关闭步进电机
//	PSTR("M84");
	enquecommand_P((PSTR("M84"))); // close
}
static void lcd_Change_filament()
{
	//加热挤出头
	setTargetHotend0(190);
	lcd_return_to_status();
//	unsigned long mytime;
//	mytime=millis()+60*1000;
//	for(;millis()>mytime;);
	
			//归原点
		enquecommand_P((PSTR("G28"))); // move all axis home
//	lcd_autohome();
//	if(current_position[Z_AXIS]<=0)
		enquecommand_P((PSTR("G1 X120 Y0 Z50")));
//	current_position[Z_AXIS]=50;
//	current_position[X_AXIS]=100;
//	plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] , current_position[E_AXIS], manual_feedrate[X_AXIS]/60, active_extruder);
	
}
static void lcd_cooldown()
{
    setTargetHotend0(0);
    setTargetHotend1(0);
    setTargetHotend2(0);
    setTargetBed(0);
    lcd_return_to_status();
//	enquecommand_P(PSTR("G 4"));
}

#ifdef BABYSTEPPING
static void lcd_babystep_x()
{
    if (encoderPosition != 0)
    {
        babystepsTodo[X_AXIS]+=(int)encoderPosition;
        encoderPosition=0;
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Babystepping X"),"");
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_tune_menu;
        encoderPosition = 0;
    }
}

static void lcd_babystep_y()
{
    if (encoderPosition != 0)
    {
        babystepsTodo[Y_AXIS]+=(int)encoderPosition;
        encoderPosition=0;
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Babystepping Y"),"");
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_tune_menu;
        encoderPosition = 0;
    }
}

static void lcd_babystep_z()
{
    if (encoderPosition != 0)
    {
        babystepsTodo[Z_AXIS]+=BABYSTEP_Z_MULTIPLICATOR*(int)encoderPosition;
        encoderPosition=0;
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Babystepping Z"),"");
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_tune_menu;
        encoderPosition = 0;
    }
}
#endif //BABYSTEPPING

static void lcd_tune_menu()
{
    START_MENU();
    MENU_ITEM(back, MSG_MAIN, lcd_main_menu);
    MENU_ITEM_EDIT(int3, MSG_SPEED, &feedmultiply, 10, 999);
    MENU_ITEM_EDIT(int3, MSG_NOZZLE, &target_temperature[0], 0, HEATER_0_MAXTEMP - 15);
#if TEMP_SENSOR_1 != 0
    MENU_ITEM_EDIT(int3, MSG_NOZZLE1, &target_temperature[1], 0, HEATER_1_MAXTEMP - 15);
#endif
#if TEMP_SENSOR_2 != 0
    MENU_ITEM_EDIT(int3, MSG_NOZZLE2, &target_temperature[2], 0, HEATER_2_MAXTEMP - 15);
#endif
#if TEMP_SENSOR_BED != 0
    MENU_ITEM_EDIT(int3, MSG_BED, &target_temperature_bed, 0, BED_MAXTEMP - 15);
#endif
    MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &fanSpeed, 0, 255);
    MENU_ITEM_EDIT(int3, MSG_FLOW, &extrudemultiply, 10, 999);

#ifdef BABYSTEPPING
    #ifdef BABYSTEP_XY
      MENU_ITEM(submenu, "Babystep X", lcd_babystep_x);
      MENU_ITEM(submenu, "Babystep Y", lcd_babystep_y);
    #endif //BABYSTEP_XY
    MENU_ITEM(submenu, "Babystep Z", lcd_babystep_z);
#endif
#ifdef FILAMENTCHANGEENABLE
     MENU_ITEM(gcode, MSG_FILAMENTCHANGE, PSTR("M600"));
#endif
    END_MENU();
}

static void lcd_prepare_menu()
{
    START_MENU();
    MENU_ITEM(back, MSG_MAIN, lcd_main_menu);
#ifdef SDSUPPORT
    #ifdef MENU_ADDAUTOSTART
      MENU_ITEM(function, MSG_AUTOSTART, lcd_autostart_sd);
    #endif
#endif
    MENU_ITEM(gcode, MSG_DISABLE_STEPPERS, PSTR("M84"));
    MENU_ITEM(function, MSG_AUTO_HOME, lcd_autohome);
    //MENU_ITEM(gcode, MSG_SET_ORIGIN, PSTR("G92 X0 Y0 Z0"));
	MENU_ITEM(function, MSG_FILAMENTCHANGE, lcd_Change_filament);
    MENU_ITEM(function, MSG_PREHEAT_PLA, lcd_preheat_pla);
    MENU_ITEM(function, MSG_PREHEAT_ABS, lcd_preheat_abs);
    MENU_ITEM(function, MSG_COOLDOWN, lcd_cooldown);
#if PS_ON_PIN > -1
    if (powersupply)
    {
        MENU_ITEM(gcode, MSG_SWITCH_PS_OFF, PSTR("M81"));
    }else{
        MENU_ITEM(gcode, MSG_SWITCH_PS_ON, PSTR("M80"));
    }
#endif
    MENU_ITEM(submenu, MSG_MOVE_AXIS, lcd_move_menu);
    END_MENU();
}

float move_menu_scale;
static void lcd_move_menu_axis();

static void lcd_move_x()
{
    if (encoderPosition != 0)
    {
        current_position[X_AXIS] += float((int)encoderPosition) * move_menu_scale;
        if (min_software_endstops && current_position[X_AXIS] < X_MIN_POS)
            current_position[X_AXIS] = X_MIN_POS;
        if (max_software_endstops && current_position[X_AXIS] > X_MAX_POS)
            current_position[X_AXIS] = X_MAX_POS;
        encoderPosition = 0;
        #ifdef DELTA
        calculate_delta(current_position);
        plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS], manual_feedrate[X_AXIS]/60, active_extruder);
        #else
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], manual_feedrate[X_AXIS]/60, active_extruder);
        #endif
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("X"), ftostr31(current_position[X_AXIS]));
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_move_menu_axis;
        encoderPosition = 0;
    }
}
static void lcd_move_y()
{
    if (encoderPosition != 0)
    {
        current_position[Y_AXIS] += float((int)encoderPosition) * move_menu_scale;
        if (min_software_endstops && current_position[Y_AXIS] < Y_MIN_POS)
            current_position[Y_AXIS] = Y_MIN_POS;
        if (max_software_endstops && current_position[Y_AXIS] > Y_MAX_POS)
            current_position[Y_AXIS] = Y_MAX_POS;
        encoderPosition = 0;
        #ifdef DELTA
        calculate_delta(current_position);
        plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS], manual_feedrate[Y_AXIS]/60, active_extruder);
        #else
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], manual_feedrate[Y_AXIS]/60, active_extruder);
        #endif
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Y"), ftostr31(current_position[Y_AXIS]));
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_move_menu_axis;
        encoderPosition = 0;
    }
}
static void lcd_move_z()
{
    if (encoderPosition != 0)
    {
        current_position[Z_AXIS] += float((int)encoderPosition) * move_menu_scale;
        if (min_software_endstops && current_position[Z_AXIS] < Z_MIN_POS)
            current_position[Z_AXIS] = Z_MIN_POS;
        if (max_software_endstops && current_position[Z_AXIS] > Z_MAX_POS)
            current_position[Z_AXIS] = Z_MAX_POS;
        encoderPosition = 0;
        #ifdef DELTA
        calculate_delta(current_position);
        plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS], manual_feedrate[Z_AXIS]/60, active_extruder);
        #else
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], manual_feedrate[Z_AXIS]/60, active_extruder);
        #endif
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Z"), ftostr31(current_position[Z_AXIS]));
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_move_menu_axis;
        encoderPosition = 0;
    }
}
static void lcd_move_e()
{
    if (encoderPosition != 0)
    {
        current_position[E_AXIS] += float((int)encoderPosition) * move_menu_scale;
        encoderPosition = 0;
        #ifdef DELTA
        calculate_delta(current_position);
        plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS], manual_feedrate[E_AXIS]/60, active_extruder);
        #else
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], manual_feedrate[E_AXIS]/60, active_extruder);
        #endif
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Extruder"), ftostr31(current_position[E_AXIS]));
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_move_menu_axis;
        encoderPosition = 0;
    }
}

static void lcd_move_menu_axis()
{
    START_MENU();
    MENU_ITEM(back, MSG_MOVE_AXIS, lcd_move_menu);
    MENU_ITEM(submenu, "Move X", lcd_move_x);
    MENU_ITEM(submenu, "Move Y", lcd_move_y);
    if (move_menu_scale < 10.0)
    {
        MENU_ITEM(submenu, "Move Z", lcd_move_z);
        MENU_ITEM(submenu, "Extruder", lcd_move_e);
    }
    END_MENU();
}

static void lcd_move_menu_10mm()
{
    move_menu_scale = 10.0;
    lcd_move_menu_axis();
}
static void lcd_move_menu_1mm()
{
    move_menu_scale = 1.0;
    lcd_move_menu_axis();
}
static void lcd_move_menu_01mm()
{
    move_menu_scale = 0.1;
    lcd_move_menu_axis();
}

static void lcd_move_menu()
{
    START_MENU();
    MENU_ITEM(back, MSG_PREPARE, lcd_prepare_menu);
    MENU_ITEM(submenu, "Move 10mm", lcd_move_menu_10mm);
    MENU_ITEM(submenu, "Move 1mm", lcd_move_menu_1mm);
    MENU_ITEM(submenu, "Move 0.1mm", lcd_move_menu_01mm);
    //TODO:X,Y,Z,E
    END_MENU();
}

static void lcd_control_menu()
{
    START_MENU();
    MENU_ITEM(back, MSG_MAIN, lcd_main_menu);
    MENU_ITEM(submenu, MSG_TEMPERATURE, lcd_control_temperature_menu);
    MENU_ITEM(submenu, MSG_MOTION, lcd_control_motion_menu);
#ifdef DOGLCD && !defined(MINIPANEL)
#if !defined(MINIPANEL)
//    MENU_ITEM_EDIT(int3, MSG_CONTRAST, &lcd_contrast, 0, 63);
    MENU_ITEM(submenu, MSG_CONTRAST, lcd_set_contrast);
#endif
#endif
#ifdef FWRETRACT
    MENU_ITEM(submenu, MSG_RETRACT, lcd_control_retract_menu);
#endif
#ifdef EEPROM_SETTINGS
    MENU_ITEM(function, MSG_STORE_EPROM, Config_StoreSettings);
    MENU_ITEM(function, MSG_LOAD_EPROM, Config_RetrieveSettings);
#endif
    MENU_ITEM(function, MSG_RESTORE_FAILSAFE, Config_ResetDefault);
    END_MENU();
}

static void lcd_control_temperature_menu()
{
#ifdef PIDTEMP
    // set up temp variables - undo the default scaling
    raw_Ki = unscalePID_i(Ki);
    raw_Kd = unscalePID_d(Kd);
#endif

    START_MENU();
    MENU_ITEM(back, MSG_CONTROL, lcd_control_menu);
    MENU_ITEM_EDIT(int3, MSG_NOZZLE, &target_temperature[0], 0, HEATER_0_MAXTEMP - 15);
#if TEMP_SENSOR_1 != 0
    MENU_ITEM_EDIT(int3, MSG_NOZZLE1, &target_temperature[1], 0, HEATER_1_MAXTEMP - 15);
#endif
#if TEMP_SENSOR_2 != 0
    MENU_ITEM_EDIT(int3, MSG_NOZZLE2, &target_temperature[2], 0, HEATER_2_MAXTEMP - 15);
#endif
#if TEMP_SENSOR_BED != 0
    MENU_ITEM_EDIT(int3, MSG_BED, &target_temperature_bed, 0, BED_MAXTEMP - 15);
#endif
    MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &fanSpeed, 0, 255);
#ifdef AUTOTEMP
    MENU_ITEM_EDIT(bool, MSG_AUTOTEMP, &autotemp_enabled);
    MENU_ITEM_EDIT(float3, MSG_MIN, &autotemp_min, 0, HEATER_0_MAXTEMP - 15);
    MENU_ITEM_EDIT(float3, MSG_MAX, &autotemp_max, 0, HEATER_0_MAXTEMP - 15);
    MENU_ITEM_EDIT(float32, MSG_FACTOR, &autotemp_factor, 0.0, 1.0);
#endif
#ifdef PIDTEMP
    MENU_ITEM_EDIT(float52, MSG_PID_P, &Kp, 1, 9990);
    // i is typically a small value so allows values below 1
    MENU_ITEM_EDIT_CALLBACK(float52, MSG_PID_I, &raw_Ki, 0.01, 9990, copy_and_scalePID_i);
    MENU_ITEM_EDIT_CALLBACK(float52, MSG_PID_D, &raw_Kd, 1, 9990, copy_and_scalePID_d);
# ifdef PID_ADD_EXTRUSION_RATE
    MENU_ITEM_EDIT(float3, MSG_PID_C, &Kc, 1, 9990);
# endif//PID_ADD_EXTRUSION_RATE
#endif//PIDTEMP
    MENU_ITEM(submenu, MSG_PREHEAT_PLA_SETTINGS, lcd_control_temperature_preheat_pla_settings_menu);
    MENU_ITEM(submenu, MSG_PREHEAT_ABS_SETTINGS, lcd_control_temperature_preheat_abs_settings_menu);
    END_MENU();
}

static void lcd_control_temperature_preheat_pla_settings_menu()
{
    START_MENU();
    MENU_ITEM(back, MSG_TEMPERATURE, lcd_control_temperature_menu);
    MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &plaPreheatFanSpeed, 0, 255);
    MENU_ITEM_EDIT(int3, MSG_NOZZLE, &plaPreheatHotendTemp, 0, HEATER_0_MAXTEMP - 15);
#if TEMP_SENSOR_BED != 0
    MENU_ITEM_EDIT(int3, MSG_BED, &plaPreheatHPBTemp, 0, BED_MAXTEMP - 15);
#endif
#ifdef EEPROM_SETTINGS
    MENU_ITEM(function, MSG_STORE_EPROM, Config_StoreSettings);
#endif
    END_MENU();
}

static void lcd_control_temperature_preheat_abs_settings_menu()
{
    START_MENU();
    MENU_ITEM(back, MSG_TEMPERATURE, lcd_control_temperature_menu);
    MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &absPreheatFanSpeed, 0, 255);
    MENU_ITEM_EDIT(int3, MSG_NOZZLE, &absPreheatHotendTemp, 0, HEATER_0_MAXTEMP - 15);
#if TEMP_SENSOR_BED != 0
    MENU_ITEM_EDIT(int3, MSG_BED, &absPreheatHPBTemp, 0, BED_MAXTEMP - 15);
#endif
#ifdef EEPROM_SETTINGS
    MENU_ITEM(function, MSG_STORE_EPROM, Config_StoreSettings);
#endif
    END_MENU();
}

static void lcd_control_motion_menu()
{
    START_MENU();
    MENU_ITEM(back, MSG_CONTROL, lcd_control_menu);
    MENU_ITEM_EDIT(float32, MSG_ZPROBE_ZOFFSET, &zprobe_zoffset, 0.5, 50);
    MENU_ITEM_EDIT(float5, MSG_ACC, &acceleration, 500, 99000);
    MENU_ITEM_EDIT(float3, MSG_VXY_JERK, &max_xy_jerk, 1, 990);
    MENU_ITEM_EDIT(float52, MSG_VZ_JERK, &max_z_jerk, 0.1, 990);
    MENU_ITEM_EDIT(float3, MSG_VE_JERK, &max_e_jerk, 1, 990);
    MENU_ITEM_EDIT(float3, MSG_VMAX MSG_X, &max_feedrate[X_AXIS], 1, 999);
    MENU_ITEM_EDIT(float3, MSG_VMAX MSG_Y, &max_feedrate[Y_AXIS], 1, 999);
    MENU_ITEM_EDIT(float3, MSG_VMAX MSG_Z, &max_feedrate[Z_AXIS], 1, 999);
    MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E, &max_feedrate[E_AXIS], 1, 999);
    MENU_ITEM_EDIT(float3, MSG_VMIN, &minimumfeedrate, 0, 999);
    MENU_ITEM_EDIT(float3, MSG_VTRAV_MIN, &mintravelfeedrate, 0, 999);
    MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_X, &max_acceleration_units_per_sq_second[X_AXIS], 100, 99000, reset_acceleration_rates);
    MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_Y, &max_acceleration_units_per_sq_second[Y_AXIS], 100, 99000, reset_acceleration_rates);
    MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_Z, &max_acceleration_units_per_sq_second[Z_AXIS], 100, 99000, reset_acceleration_rates);
    MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E, &max_acceleration_units_per_sq_second[E_AXIS], 100, 99000, reset_acceleration_rates);
    MENU_ITEM_EDIT(float5, MSG_A_RETRACT, &retract_acceleration, 100, 99000);
    MENU_ITEM_EDIT(float52, MSG_XSTEPS, &axis_steps_per_unit[X_AXIS], 5, 9999);
    MENU_ITEM_EDIT(float52, MSG_YSTEPS, &axis_steps_per_unit[Y_AXIS], 5, 9999);
    MENU_ITEM_EDIT(float51, MSG_ZSTEPS, &axis_steps_per_unit[Z_AXIS], 5, 9999);
    MENU_ITEM_EDIT(float51, MSG_ESTEPS, &axis_steps_per_unit[E_AXIS], 5, 9999);
#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
    MENU_ITEM_EDIT(bool, "Endstop abort", &abort_on_endstop_hit);
#endif
    END_MENU();
}

#ifdef DOGLCD
#if !defined(MINIPANEL)
static void lcd_set_contrast()
{
    if (encoderPosition != 0)
    {
        lcd_contrast -= encoderPosition;
        if (lcd_contrast < 0) lcd_contrast = 0;
        else if (lcd_contrast > 63) lcd_contrast = 63;
        encoderPosition = 0;
        lcdDrawUpdate = 1;
        u8g.setContrast(lcd_contrast);
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Contrast"), itostr2(lcd_contrast));
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_control_menu;
        encoderPosition = 0;
    }
}
#endif
#endif

#ifdef FWRETRACT
static void lcd_control_retract_menu()
{
    START_MENU();
    MENU_ITEM(back, MSG_CONTROL, lcd_control_menu);
    MENU_ITEM_EDIT(bool, MSG_AUTORETRACT, &autoretract_enabled);
    MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT, &retract_length, 0, 100);
    MENU_ITEM_EDIT(float3, MSG_CONTROL_RETRACTF, &retract_feedrate, 1, 999);
    MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT_ZLIFT, &retract_zlift, 0, 999);
    MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT_RECOVER, &retract_recover_length, 0, 100);
    MENU_ITEM_EDIT(float3, MSG_CONTROL_RETRACT_RECOVERF, &retract_recover_feedrate, 1, 999);
    END_MENU();
}
#endif

#if SDCARDDETECT == -1
static void lcd_sd_refresh()
{
    card.initsd();
    currentMenuViewOffset = 0;
}
#endif
static void lcd_sd_updir()
{
    card.updir();
    currentMenuViewOffset = 0;
}

void lcd_sdcard_menu()
{
    if (lcdDrawUpdate == 0 && LCD_CLICKED == 0)
        return;	// nothing to do (so don't thrash the SD card)
    uint16_t fileCnt = card.getnrfilenames();
    START_MENU();
    MENU_ITEM(back, MSG_MAIN, lcd_main_menu);
    card.getWorkDirName();
    if(card.filename[0]=='/')
    {
#if SDCARDDETECT == -1
        MENU_ITEM(function, LCD_STR_REFRESH MSG_REFRESH, lcd_sd_refresh);
#endif
    }else{
        MENU_ITEM(function, LCD_STR_FOLDER "..", lcd_sd_updir);
    }

    for(uint16_t i=0;i<fileCnt;i++)
    {
        if (_menuItemNr == _lineNr)
        {
            #ifndef SDCARD_RATHERRECENTFIRST
              card.getfilename(i);
            #else
              card.getfilename(fileCnt-1-i);
            #endif
            if (card.filenameIsDir)
            {
                MENU_ITEM(sddirectory, MSG_CARD_MENU, card.filename, card.longFilename);
            }else{
                MENU_ITEM(sdfile, MSG_CARD_MENU, card.filename, card.longFilename);
            }
        }else{
            MENU_ITEM_DUMMY();
        }
    }
    END_MENU();
}

#define menu_edit_type(_type, _name, _strFunc, scale) \
    void menu_edit_ ## _name () \
    { \
        if ((int32_t)encoderPosition < minEditValue) \
            encoderPosition = minEditValue; \
        if ((int32_t)encoderPosition > maxEditValue) \
            encoderPosition = maxEditValue; \
        if (lcdDrawUpdate) \
            lcd_implementation_drawedit(editLabel, _strFunc(((_type)encoderPosition) / scale)); \
        if (LCD_CLICKED) \
        { \
            *((_type*)editValue) = ((_type)encoderPosition) / scale; \
            lcd_quick_feedback(); \
            currentMenu = prevMenu; \
            encoderPosition = prevEncoderPosition; \
        } \
    } \
    void menu_edit_callback_ ## _name () \
    { \
        if ((int32_t)encoderPosition < minEditValue) \
            encoderPosition = minEditValue; \
        if ((int32_t)encoderPosition > maxEditValue) \
            encoderPosition = maxEditValue; \
        if (lcdDrawUpdate) \
            lcd_implementation_drawedit(editLabel, _strFunc(((_type)encoderPosition) / scale)); \
        if (LCD_CLICKED) \
        { \
            *((_type*)editValue) = ((_type)encoderPosition) / scale; \
            lcd_quick_feedback(); \
            currentMenu = prevMenu; \
            encoderPosition = prevEncoderPosition; \
            (*callbackFunc)();\
        } \
    } \
    static void menu_action_setting_edit_ ## _name (const char* pstr, _type* ptr, _type minValue, _type maxValue) \
    { \
        prevMenu = currentMenu; \
        prevEncoderPosition = encoderPosition; \
         \
        lcdDrawUpdate = 2; \
        currentMenu = menu_edit_ ## _name; \
         \
        editLabel = pstr; \
        editValue = ptr; \
        minEditValue = minValue * scale; \
        maxEditValue = maxValue * scale; \
        encoderPosition = (*ptr) * scale; \
    }\
    static void menu_action_setting_edit_callback_ ## _name (const char* pstr, _type* ptr, _type minValue, _type maxValue, menuFunc_t callback) \
    { \
        prevMenu = currentMenu; \
        prevEncoderPosition = encoderPosition; \
         \
        lcdDrawUpdate = 2; \
        currentMenu = menu_edit_callback_ ## _name; \
         \
        editLabel = pstr; \
        editValue = ptr; \
        minEditValue = minValue * scale; \
        maxEditValue = maxValue * scale; \
        encoderPosition = (*ptr) * scale; \
        callbackFunc = callback;\
    }
menu_edit_type(int, int3, itostr3, 1)
menu_edit_type(float, float3, ftostr3, 1)
menu_edit_type(float, float32, ftostr32, 100)
menu_edit_type(float, float5, ftostr5, 0.01)
menu_edit_type(float, float51, ftostr51, 10)
menu_edit_type(float, float52, ftostr52, 100)
menu_edit_type(unsigned long, long5, ftostr5, 0.01)

#ifdef REPRAPWORLD_KEYPAD
	static void reprapworld_keypad_move_z_up() {
    encoderPosition = 1;
    move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
		lcd_move_z();
  }
	static void reprapworld_keypad_move_z_down() {
    encoderPosition = -1;
    move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
		lcd_move_z();
  }
	static void reprapworld_keypad_move_x_left() {
    encoderPosition = -1;
    move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
		lcd_move_x();
  }
	static void reprapworld_keypad_move_x_right() {
    encoderPosition = 1;
    move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
		lcd_move_x();
	}
	static void reprapworld_keypad_move_y_down() {
    encoderPosition = 1;
    move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
		lcd_move_y();
	}
	static void reprapworld_keypad_move_y_up() {
		encoderPosition = -1;
		move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
    lcd_move_y();
	}
	static void reprapworld_keypad_move_home() {
		enquecommand_P((PSTR("G28"))); // move all axis home
	}
#endif

/** End of menus **/

static void lcd_quick_feedback()
{
    lcdDrawUpdate = 2;
    blocking_enc = millis() + 500;
    lcd_implementation_quick_feedback();
}

/** Menu action functions **/
static void menu_action_back(menuFunc_t data)
{
    currentMenu = data;
    encoderPosition = 0;
}
static void menu_action_submenu(menuFunc_t data)
{
    currentMenu = data;
    encoderPosition = 0;
}
static void menu_action_gcode(const char* pgcode)
{
    enquecommand_P(pgcode);
}
static void menu_action_function(menuFunc_t data)
{
    (*data)();
}
static void menu_action_sdfile(const char* filename, char* longFilename)
{
    char cmd[30];
    char* c;
    sprintf_P(cmd, PSTR("M23 %s"), filename);
    for(c = &cmd[4]; *c; c++)
        *c = tolower(*c);
    enquecommand(cmd);
    enquecommand_P(PSTR("M24"));
    lcd_return_to_status();
}
static void menu_action_sddirectory(const char* filename, char* longFilename)
{
    card.chdir(filename);
    encoderPosition = 0;
}
static void menu_action_setting_edit_bool(const char* pstr, bool* ptr)
{
    *ptr = !(*ptr);
}
#endif//ULTIPANEL

/** LCD API **/
void lcd_init()
{
    lcd_implementation_init();

#ifdef NEWPANEL
    pinMode(BTN_EN1,INPUT);
    pinMode(BTN_EN2,INPUT);
    WRITE(BTN_EN1,HIGH);
    WRITE(BTN_EN2,HIGH);
  #if BTN_ENC > 0
    pinMode(BTN_ENC,INPUT);
    WRITE(BTN_ENC,HIGH);
  #endif
  #ifdef REPRAPWORLD_KEYPAD
    pinMode(SHIFT_CLK,OUTPUT);
    pinMode(SHIFT_LD,OUTPUT);
    pinMode(SHIFT_OUT,INPUT);
    WRITE(SHIFT_OUT,HIGH);
    WRITE(SHIFT_LD,HIGH);
  #endif
#else  // Not NEWPANEL
  #ifdef SR_LCD_2W_NL // Non latching 2 wire shiftregister
     pinMode (SR_DATA_PIN, OUTPUT);
     pinMode (SR_CLK_PIN, OUTPUT);
  #elif defined(SHIFT_CLK) 
     pinMode(SHIFT_CLK,OUTPUT);
     pinMode(SHIFT_LD,OUTPUT);
     pinMode(SHIFT_EN,OUTPUT);
     pinMode(SHIFT_OUT,INPUT);
     WRITE(SHIFT_OUT,HIGH);
     WRITE(SHIFT_LD,HIGH);
     WRITE(SHIFT_EN,LOW);
  #else
     #ifdef ULTIPANEL
     #error ULTIPANEL requires an encoder
     #endif
  #endif // SR_LCD_2W_NL
#endif//!NEWPANEL

#if defined (SDSUPPORT) && defined(SDCARDDETECT) && (SDCARDDETECT > 0)
    pinMode(SDCARDDETECT,INPUT);
    WRITE(SDCARDDETECT, HIGH);
    lcd_oldcardstatus = IS_SD_INSERTED;
#endif//(SDCARDDETECT > 0)
#ifdef LCD_HAS_SLOW_BUTTONS
    slow_buttons = 0;
#endif
    lcd_buttons_update();
#ifdef ULTIPANEL
    encoderDiff = 0;
#endif
}
uint8_t tmp1_extruder;
void lcd_update()
{
	//如果温度高过60度,风扇转动
	if(degHotend(tmp1_extruder)>60)		//温度检测
	{
		WRITE(HEATER_1_PIN,OUTPUT);		//打开
		//enable_z();Z_ENABLE_PIN
	//	WRITE(Z_ENABLE_PIN,INPUT);	
	}
	else
	{
		WRITE(HEATER_1_PIN,INPUT);		//关闭
	//	WRITE(Z_ENABLE_PIN,OUTPUT);
		//disable_z();
	}

    static unsigned long timeoutToStatus = 0;

    #ifdef LCD_HAS_SLOW_BUTTONS
    slow_buttons = lcd_implementation_read_slow_buttons(); // buttons which take too long to read in interrupt context
    #endif

    lcd_buttons_update();

    #if (SDCARDDETECT > 0)
    if((IS_SD_INSERTED != lcd_oldcardstatus))
    {
        lcdDrawUpdate = 2;
        lcd_oldcardstatus = IS_SD_INSERTED;
        lcd_implementation_init(); // to maybe revive the lcd if static electricty killed it.

        if(lcd_oldcardstatus)
        {
            card.initsd();
            LCD_MESSAGEPGM(MSG_SD_INSERTED);
        }
        else
        {
            card.release();
            LCD_MESSAGEPGM(MSG_SD_REMOVED);
        }
    }
    #endif//CARDINSERTED

    if (lcd_next_update_millis < millis())
    {
#ifdef ULTIPANEL
		#ifdef REPRAPWORLD_KEYPAD
        	if (REPRAPWORLD_KEYPAD_MOVE_Z_UP) {
        		reprapworld_keypad_move_z_up();
        	}
        	if (REPRAPWORLD_KEYPAD_MOVE_Z_DOWN) {
        		reprapworld_keypad_move_z_down();
        	}
        	if (REPRAPWORLD_KEYPAD_MOVE_X_LEFT) {
        		reprapworld_keypad_move_x_left();
        	}
        	if (REPRAPWORLD_KEYPAD_MOVE_X_RIGHT) {
        		reprapworld_keypad_move_x_right();
        	}
        	if (REPRAPWORLD_KEYPAD_MOVE_Y_DOWN) {
        		reprapworld_keypad_move_y_down();
        	}
        	if (REPRAPWORLD_KEYPAD_MOVE_Y_UP) {
        		reprapworld_keypad_move_y_up();
        	}
        	if (REPRAPWORLD_KEYPAD_MOVE_HOME) {
        		reprapworld_keypad_move_home();
        	}
		#endif
        if (abs(encoderDiff) >= ENCODER_PULSES_PER_STEP)
        {
            lcdDrawUpdate = 1;
            encoderPosition += encoderDiff / ENCODER_PULSES_PER_STEP;
            encoderDiff = 0;
            timeoutToStatus = millis() + LCD_TIMEOUT_TO_STATUS;
        }
        if (LCD_CLICKED)
            timeoutToStatus = millis() + LCD_TIMEOUT_TO_STATUS;
#endif//ULTIPANEL

#ifdef DOGLCD        // Changes due to different driver architecture of the DOGM display
        blink++;     // Variable for fan animation and alive dot
        u8g.firstPage();
        do
        {
#if LANGUAGE_CHOICE == 10
            u8g.setFont(chinese);
#else
            u8g.setFont(u8g_font_6x10_marlin);
#endif
            u8g.setPrintPos(125,0);
            if (blink % 2) u8g.setColorIndex(1); else u8g.setColorIndex(0); // Set color for the alive dot
            u8g.drawPixel(127,63); // draw alive dot
            u8g.setColorIndex(1); // black on white
            (*currentMenu)();
            if (!lcdDrawUpdate)  break; // Terminate display update, when nothing new to draw. This must be done before the last dogm.next()
        } while( u8g.nextPage() );
#else
        (*currentMenu)();
#endif

#ifdef LCD_HAS_STATUS_INDICATORS
        lcd_implementation_update_indicators();
#endif

#ifdef ULTIPANEL
        if(timeoutToStatus < millis() && currentMenu != lcd_status_screen)
        {
            lcd_return_to_status();
            lcdDrawUpdate = 2;
        }
#endif//ULTIPANEL
        if (lcdDrawUpdate == 2)
            lcd_implementation_clear();
        if (lcdDrawUpdate)
            lcdDrawUpdate--;
        lcd_next_update_millis = millis() + 100;
    }
}

void lcd_setstatus(const char* message)
{
    if (lcd_status_message_level > 0)
        return;
    strncpy(lcd_status_message, message, LCD_WIDTH);
    lcdDrawUpdate = 2;
}
void lcd_setstatuspgm(const char* message)
{
    if (lcd_status_message_level > 0)
        return;
    strncpy_P(lcd_status_message, message, LCD_WIDTH);
    lcdDrawUpdate = 2;
}
void lcd_setalertstatuspgm(const char* message)
{
    lcd_setstatuspgm(message);
    lcd_status_message_level = 1;
#ifdef ULTIPANEL
    lcd_return_to_status();
#endif//ULTIPANEL
}
void lcd_reset_alert_level()
{
    lcd_status_message_level = 0;
}

#ifdef DOGLCD
#if !defined(MINIPANEL)
void lcd_setcontrast(uint8_t value)
{
    lcd_contrast = value & 63;
    u8g.setContrast(lcd_contrast);
}
#endif
#endif

#ifdef ULTIPANEL
/* Warning: This function is called from interrupt context */
void lcd_buttons_update()
{
#ifdef NEWPANEL
    uint8_t newbutton=0;
    if(READ(BTN_EN1)==0)  newbutton|=EN_A;
    if(READ(BTN_EN2)==0)  newbutton|=EN_B;
  #if BTN_ENC > 0
    if((blocking_enc<millis()) && (READ(BTN_ENC)==0))
        newbutton |= EN_C;
  #endif
    buttons = newbutton;
    #ifdef LCD_HAS_SLOW_BUTTONS
    buttons |= slow_buttons;
    #endif
    #ifdef REPRAPWORLD_KEYPAD
      // for the reprapworld_keypad
      uint8_t newbutton_reprapworld_keypad=0;
      WRITE(SHIFT_LD,LOW);
      WRITE(SHIFT_LD,HIGH);
      for(int8_t i=0;i<8;i++) {
          newbutton_reprapworld_keypad = newbutton_reprapworld_keypad>>1;
          if(READ(SHIFT_OUT))
              newbutton_reprapworld_keypad|=(1<<7);
          WRITE(SHIFT_CLK,HIGH);
          WRITE(SHIFT_CLK,LOW);
      }
      buttons_reprapworld_keypad=~newbutton_reprapworld_keypad; //invert it, because a pressed switch produces a logical 0
	#endif
#else   //read it from the shift register
    uint8_t newbutton=0;
    WRITE(SHIFT_LD,LOW);
    WRITE(SHIFT_LD,HIGH);
    unsigned char tmp_buttons=0;
    for(int8_t i=0;i<8;i++)
    {
        newbutton = newbutton>>1;
        if(READ(SHIFT_OUT))
            newbutton|=(1<<7);
        WRITE(SHIFT_CLK,HIGH);
        WRITE(SHIFT_CLK,LOW);
    }
    buttons=~newbutton; //invert it, because a pressed switch produces a logical 0
#endif//!NEWPANEL

    //manage encoder rotation
    uint8_t enc=0;
    if(buttons&EN_A)
        enc|=(1<<0);
    if(buttons&EN_B)
        enc|=(1<<1);
    if(enc != lastEncoderBits)
    {
        switch(enc)
        {
        case encrot0:
            if(lastEncoderBits==encrot3)
                encoderDiff++;
            else if(lastEncoderBits==encrot1)
                encoderDiff--;
            break;
        case encrot1:
            if(lastEncoderBits==encrot0)
                encoderDiff++;
            else if(lastEncoderBits==encrot2)
                encoderDiff--;
            break;
        case encrot2:
            if(lastEncoderBits==encrot1)
                encoderDiff++;
            else if(lastEncoderBits==encrot3)
                encoderDiff--;
            break;
        case encrot3:
            if(lastEncoderBits==encrot2)
                encoderDiff++;
            else if(lastEncoderBits==encrot0)
                encoderDiff--;
            break;
        }
    }
    lastEncoderBits = enc;
}

void lcd_buzz(long duration, uint16_t freq)
{
#ifdef LCD_USE_I2C_BUZZER
  lcd.buzz(duration,freq);
#endif
}

bool lcd_clicked()
{
  return LCD_CLICKED;
}
#endif//ULTIPANEL

/********************************/
/** Float conversion utilities **/
/********************************/
//  convert float to string with +123.4 format
char conv[8];
char *ftostr3(const float &x)
{
  return itostr3((int)x);
}

char *itostr2(const uint8_t &x)
{
  //sprintf(conv,"%5.1f",x);
  int xx=x;
  conv[0]=(xx/10)%10+'0';
  conv[1]=(xx)%10+'0';
  conv[2]=0;
  return conv;
}

//  convert float to string with +123.4 format
char *ftostr31(const float &x)
{
  int xx=x*10;
  conv[0]=(xx>=0)?'+':'-';
  xx=abs(xx);
  conv[1]=(xx/1000)%10+'0';
  conv[2]=(xx/100)%10+'0';
  conv[3]=(xx/10)%10+'0';
  conv[4]='.';
  conv[5]=(xx)%10+'0';
  conv[6]=0;
  return conv;
}

//  convert float to string with 123.4 format
char *ftostr31ns(const float &x)
{
  int xx=x*10;
  //conv[0]=(xx>=0)?'+':'-';
  xx=abs(xx);
  conv[0]=(xx/1000)%10+'0';
  conv[1]=(xx/100)%10+'0';
  conv[2]=(xx/10)%10+'0';
  conv[3]='.';
  conv[4]=(xx)%10+'0';
  conv[5]=0;
  return conv;
}

char *ftostr32(const float &x)
{
  long xx=x*100;
  if (xx >= 0)
    conv[0]=(xx/10000)%10+'0';
  else
    conv[0]='-';
  xx=abs(xx);
  conv[1]=(xx/1000)%10+'0';
  conv[2]=(xx/100)%10+'0';
  conv[3]='.';
  conv[4]=(xx/10)%10+'0';
  conv[5]=(xx)%10+'0';
  conv[6]=0;
  return conv;
}

char *itostr31(const int &xx)
{
  conv[0]=(xx>=0)?'+':'-';
  conv[1]=(xx/1000)%10+'0';
  conv[2]=(xx/100)%10+'0';
  conv[3]=(xx/10)%10+'0';
  conv[4]='.';
  conv[5]=(xx)%10+'0';
  conv[6]=0;
  return conv;
}

char *itostr3(const int &xx)
{
  if (xx >= 100)
    conv[0]=(xx/100)%10+'0';
  else
    conv[0]=' ';
  if (xx >= 10)
    conv[1]=(xx/10)%10+'0';
  else
    conv[1]=' ';
  conv[2]=(xx)%10+'0';
  conv[3]=0;
  return conv;
}

char *itostr3left(const int &xx)
{
  if (xx >= 100)
  {
    conv[0]=(xx/100)%10+'0';
    conv[1]=(xx/10)%10+'0';
    conv[2]=(xx)%10+'0';
    conv[3]=0;
  }
  else if (xx >= 10)
  {
    conv[0]=(xx/10)%10+'0';
    conv[1]=(xx)%10+'0';
    conv[2]=0;
  }
  else
  {
    conv[0]=(xx)%10+'0';
    conv[1]=0;
  }
  return conv;
}

char *itostr4(const int &xx)
{
  if (xx >= 1000)
    conv[0]=(xx/1000)%10+'0';
  else
    conv[0]=' ';
  if (xx >= 100)
    conv[1]=(xx/100)%10+'0';
  else
    conv[1]=' ';
  if (xx >= 10)
    conv[2]=(xx/10)%10+'0';
  else
    conv[2]=' ';
  conv[3]=(xx)%10+'0';
  conv[4]=0;
  return conv;
}

//  convert float to string with 12345 format
char *ftostr5(const float &x)
{
  long xx=abs(x);
  if (xx >= 10000)
    conv[0]=(xx/10000)%10+'0';
  else
    conv[0]=' ';
  if (xx >= 1000)
    conv[1]=(xx/1000)%10+'0';
  else
    conv[1]=' ';
  if (xx >= 100)
    conv[2]=(xx/100)%10+'0';
  else
    conv[2]=' ';
  if (xx >= 10)
    conv[3]=(xx/10)%10+'0';
  else
    conv[3]=' ';
  conv[4]=(xx)%10+'0';
  conv[5]=0;
  return conv;
}

//  convert float to string with +1234.5 format
char *ftostr51(const float &x)
{
  long xx=x*10;
  conv[0]=(xx>=0)?'+':'-';
  xx=abs(xx);
  conv[1]=(xx/10000)%10+'0';
  conv[2]=(xx/1000)%10+'0';
  conv[3]=(xx/100)%10+'0';
  conv[4]=(xx/10)%10+'0';
  conv[5]='.';
  conv[6]=(xx)%10+'0';
  conv[7]=0;
  return conv;
}

//  convert float to string with +123.45 format
char *ftostr52(const float &x)
{
  long xx=x*100;
  conv[0]=(xx>=0)?'+':'-';
  xx=abs(xx);
  conv[1]=(xx/10000)%10+'0';
  conv[2]=(xx/1000)%10+'0';
  conv[3]=(xx/100)%10+'0';
  conv[4]='.';
  conv[5]=(xx/10)%10+'0';
  conv[6]=(xx)%10+'0';
  conv[7]=0;
  return conv;
}

// Callback for after editing PID i value
// grab the pid i value out of the temp variable; scale it; then update the PID driver
void copy_and_scalePID_i()
{
#ifdef PIDTEMP
  Ki = scalePID_i(raw_Ki);
  updatePID();
#endif
}

// Callback for after editing PID d value
// grab the pid d value out of the temp variable; scale it; then update the PID driver
void copy_and_scalePID_d()
{
#ifdef PIDTEMP
  Kd = scalePID_d(raw_Kd);
  updatePID();
#endif
}

#endif //ULTRA_LCD


void lcd_vs_status(char lan)
{
	
	vs_lcd_data[0]=0x5A;
	vs_lcd_data[1]=0xA5;
	vs_lcd_data[2]=0x05;
	vs_lcd_data[3]=0x82;
	vs_lcd_data[4]=0x00;
	vs_lcd_data[5]=0x05;
	vs_lcd_data[6]=0x00;
	vs_lcd_data[7]=lan;
	for(int i=0 ; i<8 ; i++)
	{
		MYSERIAL2.write(vs_lcd_data[i]);
		_delay_ms(2);
	}	
}


void lcd_vs_App_Page(char temp_page)
{

	vs_lcd_data[0]=0x5A;
	vs_lcd_data[1]=0xA5;
	vs_lcd_data[2]=0x04;
	vs_lcd_data[3]=0x80;
	vs_lcd_data[4]=0x03;

	vs_lcd_data[5]=0x00;
	vs_lcd_data[6]=temp_page;

	for(int i=0 ; i<7 ; i++)
	{
		MYSERIAL2.write(vs_lcd_data[i]);
		_delay_ms(2);
	}
}


void lcd_vs_loop_com(char comdate0,char comdate1)
{

	vs_lcd_data[0]=0x5A;
	vs_lcd_data[1]=0xA5;
	vs_lcd_data[2]=0x04;
	vs_lcd_data[3]=0x83;
	vs_lcd_data[4]=comdate0;

	vs_lcd_data[5]=comdate1;
	vs_lcd_data[6]=0x01;

	for(int i=0 ; i<7 ; i++)
	{
		MYSERIAL2.write(vs_lcd_data[i]);
		_delay_ms(2);
	}
}

void lcd_vs_command()
{

	if(MYSERIAL2.available()>0 )
	{	
		BT_DATA = "";
		while (MYSERIAL2.available()>0)
		{

			vs_temp_data = char(MYSERIAL2.read());
			BT_DATA += vs_temp_data;
			_delay_ms(2);
		}
		const char *lcd_vs_com_data;
		unsigned int len = BT_DATA.length();
	
		lcd_vs_com_data=BT_DATA.c_str();

		if(90!=lcd_vs_com_data[0] && 165!=lcd_vs_com_data[1] && 3<lcd_vs_com_data[4])
			return;
	
		switch ((int)lcd_vs_com_data[5])
		{ 
		case 50 : 
			{
				if(lcd_oldcardstatus)	
				{
					
					card.initsd();
				}
				else
				{
					card.release();
					break;
				}
			
			sdfilename[0]=0x5A;
			sdfilename[1]=0xA5;
			sdfilename[2]=0x11;
			sdfilename[3]=0x82;
			sdfilename[4]=0x01;
			sdfilename[5]=0x00;

			uint16_t fileCnt = card.getnrfilenames();
			if(file_num<1)
				file_num=fileCnt;
			card.getWorkDirName();
			char mydata4=0x03;
			char mydata5=0x01;
			if(mydata4==lcd_vs_com_data[8])
			{
				if(fileCnt>4  )
				{
					if(file_num>4)
						file_num=file_num-4;
				}
			}
			if(mydata5==lcd_vs_com_data[8])
			{
				if(fileCnt>4 )
				{
					if(file_num<=(fileCnt-4))
						file_num=file_num+4;
				}
			}

			for(uint8_t i=0;i<4;i++)
			{
				if(i==0)
					sdfilename[5]=0x00;
				if(i==1)
					sdfilename[5]=0x09;
				if(i==2)
					sdfilename[5]=0x10;
				if(i==3)
					sdfilename[5]=0x19;
				

				for(int ui=0; ui<13; ui++)
					sdfilename[ui+6]=0x00;

				if((file_num-i)>0&&(file_num-i)<60000)
				{
					card.getfilename(file_num-1-i);
					for(int yi=0 ; yi<13 ;yi++)
							sdfilename[yi+6]=card.longFilename[yi];	
					
				}

				for(int u=0; u<20 ;u++)
				{
					MYSERIAL2.write(sdfilename[u]);
					_delay_ms(2);
				}
			}

			if(acceleration>1000)
				lcd_vs_App_Page(0x47);
			else
				lcd_vs_App_Page(0x9F);
			break; 
			}

		case 51 :
			{
				if(lcd_vs_com_data[8]>check7)
					break;
				if(lcd_vs_com_data[8]>=check4)
				{	
					uint16_t sd_num1=0;
					if(check4==lcd_vs_com_data[8])
						sd_num1=file_num;
					if(check5==lcd_vs_com_data[8])
						sd_num1=file_num-1;
					if(check6==lcd_vs_com_data[8])
						sd_num1=file_num-2;
					if(check7==lcd_vs_com_data[8])
						sd_num1=file_num-3;

					if( card.cardOK )
					{
						card.getfilename(sd_num1-1);
						MYSERIAL2.println(card.filename);
						card.openFile(card.filename,true);
						card.startFileprint();	
						starttime=millis(); 
					}

					sdfilename[0]=0x5A;
					sdfilename[1]=0xA5;
					sdfilename[2]=0x11;
					sdfilename[3]=0x82;
					//LCD屏 保存文件名的变量地址
					sdfilename[4]=0x01;
					sdfilename[5]=0x20;
					//	card.getfilename(file_num-1-i);
					//清理存储空间
					for(int ui=0; ui<13; ui++)
						//  card.filename[ui]=0x00;
						sdfilename[ui+6]=0x00;
					//复制要打印的文件名
					for(int yi=0 ; yi<13 ;yi++)
					{
						sdfilename[yi+6]=card.longFilename[yi];		//如果含有字符就赋值
					}
					for(int u=0; u<20 ;u++)
					{
						MYSERIAL2.write(sdfilename[u]);
						_delay_ms(2);
					}

				}	
		//		lcd_vs_App_Page(0x41); //3.5
			//	lcd_vs_App_Page(0x32);
			//	_delay_ms(10);
				//	lcd_vs_status(0x01);
				//跳转到打印界面
				if(acceleration>1000)
					lcd_vs_App_Page(0x49);//中文
				else
					lcd_vs_App_Page(0xA1);//英文
				break; 
				
			}

		case 52:
			{	//降温
			//	enquecommand_P(PSTR("M81")); 
				setTargetHotend0(0);
				setTargetBed(0);

			//	lcd_vs_App_Page(0x30);
			//	lcd_vs_status(0x00);
				break;
			}
		case 53:
			{
				//停止打印
				card.sdprinting = false;
				card.closefile();
				quickStop();
				if(SD_FINISHED_STEPPERRELEASE)
				{
					enquecommand_P(PSTR(SD_FINISHED_RELEASECOMMAND));
				}
				autotempShutdown();

				setTargetHotend0(0);
				setTargetBed(0);
				fanSpeed=0;
			//	lcd_vs_App_Page(0x30);
			//	lcd_vs_status(0x00);
				if(acceleration>1000)
					lcd_vs_App_Page(0x45);//中文
				else
					lcd_vs_App_Page(0x9D);//英文
				break;
			}
		case 54:
			{
				//暂停打印
				card.pauseSDPrint();
				//让X,Y轴归原点
				enquecommand_P(PSTR("G1 X5 Y5"));
				//关闭X Y步进电机
			//	disable_x();
			//	disable_y();
				break;
			}
		case 55:
			{
				//让X,Y轴归原点
			//	enquecommand_P(PSTR("G28 X0 Y0")); 
				//继续打印
				card.startFileprint();
				break;
			}
		case 56:
			{
				//打印速度百分比 lcd_vs_com_data
				if(lcd_vs_com_data[8]>0)
				{
					if(feedmultiply<301)
						feedmultiply+=5;
				}
				else
				{
					if(feedmultiply>10)
						feedmultiply-=5;
				}
				break;
			}
		case 57:
			{
				
				
				if(lcd_vs_com_data[8]>0)
				{
					if(tmp_extru_tm<301)
					{
						tmp_extru_tm++;
						setTargetHotend0(tmp_extru_tm);
					}
				}
				else
				{
					if(tmp_extru_tm>0)
					{
						tmp_extru_tm--;
						setTargetHotend0(tmp_extru_tm);
					}
				}
				
				break;
			}
		case 58:
			{
				//热床加热 lcd_vs_com_data
			//	target_temperature_bed=(int)lcd_vs_com_data[8];
				if(lcd_vs_com_data[8]>0)
				{
					if(target_temperature_bed<125)
						target_temperature_bed++;
				}
				else
				{
					if(target_temperature_bed>0)
						target_temperature_bed--;
				}
				break;
			}
		case 59:
			{
				//风扇转速百分比
			//	fanSpeed=(int)lcd_vs_com_data[8]*255/100;
				if(lcd_vs_com_data[8]>0)
				{
					if(fanSpeed<255)
						fanSpeed+=3;
				}
				else
				{
					if(fanSpeed>0)
						fanSpeed-=3;
				}
				break;
			}

		case 60:
			{
				//预热Pla
		//		lcd_preheat_pla();
				setTargetHotend0(185);
				setTargetBed(50);
			//	lcd_vs_App_Page(0x30);
			//	lcd_vs_status(0x02);
				break;
			}

		case 61:
			{
				//预热ABS
		//		lcd_preheat_abs();
				setTargetHotend0(210);
				setTargetBed(70);
			//	lcd_vs_App_Page(0x30);
			//	lcd_vs_status(0x02);
				break;
			}
		case 62:
			{
				//显示 电机/PID 的设置
				//帧头
				vs_lcd_data[0]=0x5A;
				vs_lcd_data[1]=0xA5;
				//字节长度
				vs_lcd_data[2]=0x11;
				//命令
				vs_lcd_data[3]=0x82;
				//地址
				vs_lcd_data[4]=0x03;
				vs_lcd_data[5]=0x24;
				//数据
				//X轴
				vs_lcd_data[7]=axis_steps_per_unit[X_AXIS];
				if(axis_steps_per_unit[X_AXIS]>255)
					vs_lcd_data[6]=axis_steps_per_unit[X_AXIS]/255;
				else
					vs_lcd_data[6]=0x00;
				
				//Y轴
				vs_lcd_data[9]=axis_steps_per_unit[Y_AXIS];
				if(axis_steps_per_unit[Y_AXIS]>255)
					vs_lcd_data[8]=axis_steps_per_unit[Y_AXIS]/255;
				else
					vs_lcd_data[8]=0x00;
				//Z轴
				vs_lcd_data[11]=axis_steps_per_unit[Z_AXIS];
				if(axis_steps_per_unit[Z_AXIS]>255)
					vs_lcd_data[10]=axis_steps_per_unit[Z_AXIS]/255;
				else
					vs_lcd_data[10]=0x00;
				//E 轴
				vs_lcd_data[13]=axis_steps_per_unit[E_AXIS];
				if(axis_steps_per_unit[E_AXIS]>255)
					vs_lcd_data[12]=axis_steps_per_unit[E_AXIS]/255;
				else
					vs_lcd_data[12]=0x00;
				//PID - P
				vs_lcd_data[14]=0x00;
				vs_lcd_data[15]=Kp;
				//PID - I  DEFAULT_Ki
				vs_lcd_data[16]=0x00;
				vs_lcd_data[17]=unscalePID_i(Ki);
				//PID - D	DEFAULT_Kd
				vs_lcd_data[18]=0x00;
				vs_lcd_data[19]=unscalePID_d(Kd);

				for(int i=0 ; i<20 ; i++)
				{
					MYSERIAL2.write(vs_lcd_data[i]);
					_delay_ms(2);
				}


				if(lcd_vs_com_data[8]>0)
				{	//PID 页面
					//	lcd_vs_App_Page(0x49); //3.5
					//	lcd_vs_App_Page(0x47);
					//语言选择
					//检查是中英文，界面跳转
					if(acceleration>1000)
						lcd_vs_App_Page(0x54);//中文
					else
						lcd_vs_App_Page(0xAC);//英文
				}
				else
				{	//电机 页面
					//	lcd_vs_App_Page(0x47); //3.5
					//	lcd_vs_App_Page(0x45);
					//语言选择
					//检查是中英文，界面跳转
					if(acceleration>1000)
						lcd_vs_App_Page(0x56);//中文
					else
						lcd_vs_App_Page(0xAE);//英文
				}
				
				break;
			}
		case 63:
				{
					//要求返回 电机/PID 的设置
					//帧头
					vs_lcd_data[0]=0x5A;
					vs_lcd_data[1]=0xA5;
					//字节长度
					vs_lcd_data[2]=0x04;
					//命令
					vs_lcd_data[3]=0x83;
					//地址
					vs_lcd_data[4]=0x03;
					vs_lcd_data[5]=0x24;
					//返回7个整型数据
					vs_lcd_data[6]=0x07;
				
					unsigned long temp_t;
					temp_t=millis()+1000;
		
					char mychar[21];
			
					uint8_t iu=0;

					for(int i=0 ; i<7 ; i++)
					{
						MYSERIAL2.write(vs_lcd_data[i]);
						_delay_ms(2);
					}
					
					
					do
					{
						if(temp_t<= millis()) break;
						while (MYSERIAL2.available()>0)
						{
							//接收字节，转化为字符串
							mychar[iu]=char(MYSERIAL2.read());
							iu++;
							_delay_ms(2); //延时2毫秒
						}
					}while(1);
					if(iu>7)
					{
						// X Y Z E  轴
						axis_steps_per_unit[X_AXIS]=mychar[7]*255+mychar[8];
						axis_steps_per_unit[Y_AXIS]=mychar[9]*255+mychar[10];
						axis_steps_per_unit[Z_AXIS]=257;
						axis_steps_per_unit[Z_AXIS]+=mychar[11]*255+mychar[12];
						
					//	axis_steps_per_unit[Z_AXIS]+=strtod(itostr3(lcd_vs_com_data[11]*255+lcd_vs_com_data[12]),NULL);
					//	axis_steps_per_unit[Z_AXIS]+=lcd_vs_com_data[12];
						axis_steps_per_unit[E_AXIS]=mychar[13]*255+mychar[14];

						//PID 值
						Kp=mychar[15]*255+mychar[16];
						Ki=scalePID_i(mychar[17]*255+mychar[18]);
						Kd=scalePID_d(mychar[19]*255+mychar[20]);
					}

					//设置保存到EEPROM里
					Config_StoreSettings();

				
					//语言选择
					//检查是中英文，界面跳转
					if(acceleration>1000)
						lcd_vs_App_Page(0x52);//中文
					else
						lcd_vs_App_Page(0xAA);//英文
					
					
					break;
				}
		case 64:
			{
				if(acceleration>1000)
						lcd_vs_App_Page(0x49);//中文
					else
						lcd_vs_App_Page(0xA1);//英文
				//要求返回 温度和其他 的设置
					//帧头
					vs_lcd_data[0]=0x5A;
					vs_lcd_data[1]=0xA5;
					//字节长度
					vs_lcd_data[2]=0x04;
					//命令
					vs_lcd_data[3]=0x83;
					//地址
					vs_lcd_data[4]=0x03;
					vs_lcd_data[5]=0x2B;

					vs_lcd_data[6]=0x04;
				
					unsigned long temp_t;
					temp_t=millis()+600;
					//字符串
					char mychar[21];
					uint8_t iu=0;

					for(int i=0 ; i<7 ; i++)
					{
						MYSERIAL2.write(vs_lcd_data[i]);
						_delay_ms(2);
					}
					
					//等待信息发回 1秒内 超出时间自动退出
					do
					{
						if(temp_t< millis()) break;
						while (MYSERIAL2.available()>0)
						{
							//接收字节，转化为字符串
							mychar[iu]=char(MYSERIAL2.read());
							iu++;
							_delay_ms(2); //延时2毫秒
						}
					}while(1);
					if(iu>7)
					{
						char cmd1[30];
					//	unsigned int temp_target012=0;
						//速度
						uint8_t myfeed=0;
						myfeed=mychar[8];
						if( 0<mychar[7])
							feedmultiply=255+myfeed;
					//		sprintf_P(cmd1, PSTR("M220 S%i.g"), (int)(255+mychar[8]));
						else
							feedmultiply=myfeed;
					//		sprintf_P(cmd1, PSTR("M220 S%i.g"), (int)(mychar[8]));
					//	enquecommand(cmd1); 

						_delay_ms(10);
						
						//热床温度
						setTargetBed((int)mychar[12]);
						_delay_ms(10); 
						//风扇速度
						fanSpeed=(unsigned int)mychar[14]*255/100;

						//挤出头温度
						tmp_extruder12=(int)(mychar[9]*255+mychar[10]);
				/*/		setTargetHotend((float)temp_target01,0); 
				//		_delay_ms(100);
					//	setTargetHotend0(130);
					//	autotemp_enabled=false;
					//	temp_target012=130;
					//	setTargetHotend0(&temp_target012);
					//	target_temperature[0]=temp_target012;
					//	if(128<temp_target012)
					//		setTargetHotend0(140);
					//		setTargetHotend0(temp_target012);
					//	else
					//		setTargetHotend0(temp_target012);
						/*  for(int ct=0 ; ct<300; ct++)
							{
								if(ct==temp_target012)
								{
									setTargetHotend0(ct);
									break;
								}
							}*/
					//		setE0temp(temp_target012);
						sprintf_P(cmd1, PSTR("M104 T0 S%i.g"), tmp_extruder12);
						enquecommand(cmd1); 
					//	PSTR(&target_temperature[0],0);
						_delay_ms(10);
					}
					
				break;
			}

		case 65:
			{	//中英文界面切换
				if(acceleration>1000)
					acceleration=1000;
				else
					acceleration=1002;
				//设置保存到EEPROM里
				Config_StoreSettings();
				break;
			}
		case 66:
			{	//恢复出厂设置
				Config_ResetDefault();
				//设置保存到EEPROM里
				Config_StoreSettings();
				break;
			}
			case 67:
			{	//X轴归零
				enquecommand_P(PSTR("G28 X0")); // move all axis home
				//关闭步进电机
			//	enquecommand_P(PSTR("M84")); // close
				break;
			}
			case 68:
			{	//Y轴归零
				enquecommand_P(PSTR("G28 Y0")); // move all axis home
				//关闭步进电机
			//	enquecommand_P(PSTR("M84")); // close
				break;
			}
			case 69:
			{	//Z轴归零
				enquecommand_P(PSTR("G28 Z0")); // move all axis home
				//关闭步进电机
			//	enquecommand_P(PSTR("M84")); // close
				break;
			}
			case 70:
			{	//换料程序

				//加热挤出头
				setTargetHotend0(190);
			
				//归原点
				enquecommand_P((PSTR("G28"))); // move all axis home
				//到指定高度
				enquecommand_P((PSTR("G1 X120 Y0 Z50")));
				break;
			}
			case 71:
			{	

				//当前温度变成目标温度 挤出头
		//		setTargetHotend0(tmp_extru);
				
		//		tmp_extru_tm=tmp_extru;
				//当前温度变成目标温度 热床
		//		target_temperature_bed=degBed();

		//		target_temperature_bed_tm=degBed();

				//显示 电机/PID 的设置
				//帧头
				vs_lcd_data[0]=0x5A;
				vs_lcd_data[1]=0xA5;
				//字节长度
				vs_lcd_data[2]=0x0B;
				//命令
				vs_lcd_data[3]=0x82;
				//地址
				vs_lcd_data[4]=0x03;
				vs_lcd_data[5]=0x2B;
				//数据
				//打印速度
				vs_lcd_data[7]=feedmultiply;
				if(feedmultiply>255)
					vs_lcd_data[6]=feedmultiply/255;
				else
					vs_lcd_data[6]=0x00;

				//挤出头目标温度
			//	vs_lcd_data[9]=tmp_extru;
				vs_lcd_data[9]=(int)degTargetHotend(0);
				if(tmp_extru>255)
					vs_lcd_data[8]=(int)degTargetHotend(0)/255;
				else
					vs_lcd_data[8]=0x00;
				//热床目标温度
				vs_lcd_data[10]=0x00;
				vs_lcd_data[11]=(int)degTargetBed();
			//	vs_lcd_data[11]=degBed();degTargetBed()

				//风扇
				vs_lcd_data[12]=0x00;
				vs_lcd_data[13]=fanSpeed*100/255;

				for(int i=0 ; i<14 ; i++)
				{
					MYSERIAL2.write(vs_lcd_data[i]);
					_delay_ms(2);
				}

			//	_delay_ms(10);
				if(acceleration>1000)
					lcd_vs_App_Page(0x4B);//中文
				else
					lcd_vs_App_Page(0xA3);//英文

				break;
			}

			case 72 :	//自动进料
				{


					//进料
					//	enquecommand_P((PSTR("M109 S200"))); //加热

					//	enquecommand_P((PSTR("G1 F100 E200"))); //F100 转速  E200 下料200毫米

					//加热
					setTargetHotend0(200);
					if(193<=current_temperature[0]) //温度检测
					{
						//	enquecommand_P((PSTR("G1 F25 ")));
						//挤出头下料4MM
						current_position[E_AXIS]+=0.68; //下料多少毫米
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] , current_position[E_AXIS], manual_feedrate[E_AXIS]/60, active_extruder);

					}
					_delay_ms(600);	//等待时间
					//发送返回循环指令
					lcd_vs_loop_com(0x04,0x48);

					break;
				}
		case 73 :	//自动退料
			{
				//退料  先下20毫米，后快速往上退料
				//	enquecommand_P((PSTR("M109 S200"))); //加热
				//	enquecommand_P((PSTR("G1 F100 E10"))); 
				//	enquecommand_P((PSTR("G1 F10000 E-100"))); //F100 转速  E200 退料200毫米

				//加热
				setTargetHotend0(200);
				if(193<=current_temperature[0]) //温度检测
				{
				//	enquecommand_P((PSTR("G1 F10000 ")));
					//挤出头下料4MM
					current_position[E_AXIS]-=4; //退料多少毫米
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] , current_position[E_AXIS], manual_feedrate[E_AXIS], active_extruder);

				}
				_delay_ms(100); //等待时间
				//发送返回循环指令
				lcd_vs_loop_com(0x04,0x49);
				break;
			}
		case 74 :	//关闭加热
			{
				//关闭加热
				setTargetHotend0(0);
				//跳转界面
				if(acceleration>1000)
					lcd_vs_App_Page(0xC3);//中文
				else
					lcd_vs_App_Page(0xB2);//英文
				//break;
				//	enquecommand_P((PSTR("M109 S0"))); //加热	
				enquecommand_P(PSTR("M84")); 
				break;
			}
		case 75 :	//
			{
				
				//等待时间到
				if(millis()-oldtime11>6000)
				{
					if(acceleration>1000)
						lcd_vs_App_Page(0xCA);//中文
					else
						lcd_vs_App_Page(0xB9);//英文
					break;
				}
				_delay_ms(300);
				//发送返回循环指令
				lcd_vs_loop_com(0x04,0x4B);

				break;
			}
		case 76 :	//自动调平
			{
				if(0x00==lcd_vs_com_data[8]) 
				{
					//归原点
					enquecommand_P((PSTR("G28 F10000"))); // move all axis home
					oldtime11=millis();
					_delay_ms(300);
					//发送返回循环指令
					lcd_vs_loop_com(0x04,0x4B);
					
				}

				if(0x01==lcd_vs_com_data[8]) 
				{
					//第一点
					enquecommand_P((PSTR("G1 F10000 X30 Y30 Z20")));
					enquecommand_P((PSTR("G28 Z0"))); // move Z axis home
				}

				if(0x02==lcd_vs_com_data[8]) 
				{
					//第二点
					enquecommand_P((PSTR("G1 X185 Y30 Z20")));
					enquecommand_P((PSTR("G28 Z0"))); // move Z axis home
				}

				if(0x03==lcd_vs_com_data[8]) 
				{
					//第三点
					enquecommand_P((PSTR("G1 X185 Y185 Z20")));
					enquecommand_P((PSTR("G28 Z0"))); // move Z axis home
				}

				if(0x04==lcd_vs_com_data[8]) 
				{
					//第四点
					enquecommand_P((PSTR("G1 X30 Y185 Z20")));
					enquecommand_P((PSTR("G28 Z0"))); // move Z axis home
				}

				if(0x05==lcd_vs_com_data[8]) 
				{
					//点OK
					enquecommand_P((PSTR("G28 "))); // move Z axis home
					enquecommand_P((PSTR("G1 Z30"))); // move Z axis
					enquecommand_P(PSTR("M84"));
				}
				break;
			}


		case 28 :	//自动归位 Home all Axis
			{
			
				//归原点
				enquecommand_P(PSTR("G28")); // move all axis home

				//关闭步进电机
				enquecommand_P(PSTR("M84")); // close
				break;
			}
		case 1:
			{
				if(0==lcd_vs_com_data[8])
				{
					current_position[X_AXIS]+=10;
					if(current_position[X_AXIS]>X_MAX_LENGTH) break;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] , current_position[E_AXIS], manual_feedrate[X_AXIS]/60, active_extruder);
				}
				//	enquecommand_P(PSTR("G1 X10")); // X轴
				if(1==lcd_vs_com_data[8])
				{
					current_position[X_AXIS]-=10;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] , current_position[E_AXIS], manual_feedrate[X_AXIS]/60, active_extruder);
				}
				//	enquecommand_P(PSTR("G1 X-10")); // -X轴
				if(2==lcd_vs_com_data[8])
				{
					current_position[Y_AXIS]+=10;
					if(current_position[Y_AXIS] >Y_MAX_LENGTH) break;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] , current_position[E_AXIS], manual_feedrate[Y_AXIS]/60, active_extruder);
				}
				//	enquecommand_P(PSTR("G1 Y10")); // Y轴
				if(3==lcd_vs_com_data[8])
				{
					current_position[Y_AXIS]-=10;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] , current_position[E_AXIS], manual_feedrate[Y_AXIS]/60, active_extruder);
				}
				//	enquecommand_P(PSTR("G1 Y-10")); // -Y轴
				if(4==lcd_vs_com_data[8])
				{
					current_position[Z_AXIS]+=5;
					if(current_position[Z_AXIS]>Z_MAX_LENGTH) break;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] , current_position[E_AXIS], manual_feedrate[Z_AXIS]/60, active_extruder);
				}
				//	enquecommand_P(PSTR("G1 Z5")); // Z轴
				if(5==lcd_vs_com_data[8])
				{
					current_position[Z_AXIS]-=5;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] , current_position[E_AXIS], manual_feedrate[Z_AXIS]/60, active_extruder);
				}
				//	enquecommand_P(PSTR("G1 Z-5")); // -Z轴

				if(degHotend(tmp1_extruder)>180)
				{
					if(6==lcd_vs_com_data[8])
					{
					current_position[E_AXIS]-=10;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] , current_position[E_AXIS], manual_feedrate[E_AXIS]/60, active_extruder);
					}
				//		enquecommand_P(PSTR("G1 E10")); // -E轴
					if(7==lcd_vs_com_data[8])
					{
					current_position[E_AXIS]+=10;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] , current_position[E_AXIS], manual_feedrate[E_AXIS]/60, active_extruder);
					}
				//		enquecommand_P(PSTR("G1 E-10")); // E轴
					//关闭电机E轴电机
				    enquecommand_P(PSTR("M84 E"));
				}
				//enquecommand_P(PSTR("M84")); 
				break;
			}
		case 84:
			{	//关闭电机
				enquecommand_P(PSTR("M84")); 
				break;
			}
		case 81:
			{	//关闭电源
				enquecommand_P(PSTR("M81")); 
				break;
			}

		
		default:  
			break;;
		}
		
	}
	
}


void lcd_update()
{
	
	if((IS_SD_INSERTED != lcd_oldcardstatus))
    {
	lcd_oldcardstatus = IS_SD_INSERTED;
	file_num=0;
	}
	lcd_vs_command();
	if(millis()-oldtime>1000)
	{		
	lcd_vs_temp_data();
	oldtime=millis();
	}


}

void lcd_init()
{

	#if defined (SDSUPPORT) && defined(SDCARDDETECT) && (SDCARDDETECT > 0)
    pinMode(SDCARDDETECT,INPUT);
    WRITE(SDCARDDETECT, HIGH);
    lcd_oldcardstatus = IS_SD_INSERTED;
	file_num=0;
	#endif

}
void setE0temp(int tempE0)
{
	if(128==tempE0)
			setTargetHotend0(128);
		
	if(129==tempE0)
			setTargetHotend0(129);

	if(130==tempE0)
			setTargetHotend0(130);

	if(131==tempE0)
			setTargetHotend0(131);
/*		case 1:
			setTargetHotend0(129);
			break;

		case 2:
			setTargetHotend0(130);
			break;

		case 3:
			setTargetHotend0(131);
			break;

		case 4:
			setTargetHotend0(132);
			break;
		case 5:
			setTargetHotend0(133);
			break;
		case 6:
			setTargetHotend0(134);
			break;
		case 7:
			setTargetHotend0(135);
			break;
		case 8:
			setTargetHotend0(136);
			break;
		case 9:
			setTargetHotend0(137);
			break;
		case 10:
			setTargetHotend0(138);
			break;
		default:
			setTargetHotend0(142);
            break;
	}
	*/
}