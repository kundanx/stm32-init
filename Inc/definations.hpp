#ifndef _DEFINATIONS_
#define _DEFINATIONS_

#define START_BYTE (0xA5)

// Button's bit position in the byte
#define B_X (7)
#define B_Y (6)
#define B_A (5)
#define B_B (4)
#define B_UP (3)
#define B_DOWN (2)
#define B_LB (1)
#define B_RB (0)

#define B_START (7)
#define B_BACK (6)
#define B_XBOX (5)
#define B_LEFT (4)
#define B_RIGHT (3)
#define B_L3 (2)
#define B_R3 (1)

#ifndef _BV
#define _BV(x) (1 << x)
#endif

#define NUM_JOYSTICK_BYTES (8)
#define JOYSTICK_START_BYTE (START_BYTE)

struct JoystickData
{
  uint8_t button1 = 0;
  uint8_t button2 = 0;
  uint8_t lt = 0;
  uint8_t rt = 0;
  int8_t l_hatx = 0;
  int8_t l_haty = 0;
  int8_t r_hatx = 0;
  int8_t r_haty = 0;
};

#endif // !_DEFINATIONS_
