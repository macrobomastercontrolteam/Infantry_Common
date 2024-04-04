#ifndef GRAPHIC_H
#define GRAPHIC_H
#include <stdint.h>

#define UI_SOF 0xA5

// Screen resolution is 1920x1080
#define SCREEN_WIDTH 1080
#define SCREEN_LENGTH 1920

#define MAX_SIZE 128      // Maximum length of uploaded data
#define FRAMEHEADER_LEN 5 // Frame header length
#define CMD_LEN 2         // Command code length
#define CRC_LEN 2         // CRC16 checksum

/****************************CMD_ID Data ********************/
// CMD_ID in the packet header denotes inter-robot interaction data,
// triggered by the sender for transmission, with an upper limit of 10Hz.
#define UI_CMD_Robo_Exchange 0x0301

// Draw Command IDs
#define UI_Data_ID_Del 0x100
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110

// Sender IDs
#define UI_Data_RobotID_RHero 1
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9

#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109

// Receiver IDs
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106

#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A

// Default Client and Recievers
#define Default_Client_ID UI_Data_CilentID_RHero
#define Default_Robot_ID UI_Data_RobotID_RHero

/***************************Deletion Operation***************************/
#define UI_Data_Del_NoOperate 0 // No operation during deletion
#define UI_Data_Del_Layer 1     // Delete by layer
#define UI_Data_Del_ALL 2       // Delete all

/***************************Graphic Configuration Parameters - Graphic Operation********************/
#define UI_Graph_ADD 1    // Add a graphic
#define UI_Graph_Change 2 // Change a graphic
#define UI_Graph_Del 3    // Delete a graphic

/***************************Graphic Configuration Parameters - Graphic Type********************/
#define UI_Graph_Line 0      // Straight line
#define UI_Graph_Rectangle 1 // Rectangle
#define UI_Graph_Circle 2    // Whole circle
#define UI_Graph_Ellipse 3   // Ellipse
#define UI_Graph_Arc 4       // Circular arc
#define UI_Graph_Float 5     // Floating-point type
#define UI_Graph_Int 6       // Integer type
#define UI_Graph_Char 7      // Character type

/***************************Graphic Configuration Parameters - Graphic Color********************/
#define UI_Color_Main 0         // Red and blue primary color
#define UI_Color_Yellow 1       // Default yellow, also used for white, green, orange
#define UI_Color_Green 2        // Green
#define UI_Color_Orange 3       // Orange
#define UI_Color_Purplish_red 4 // Purplish-red
#define UI_Color_Pink 5         // Pink
#define UI_Color_Cyan 6         // Cyan
#define UI_Color_Black 7        // Black
#define UI_Color_White 8        // White

typedef struct
{
   uint8_t SOF;          // Start byte, fixed at 0xA5
   uint16_t data_length; // Frame data length
   uint8_t seq;          // Packet sequence number
   uint8_t CRC8;         // CRC8 checksum value
   uint16_t cmd_ID;      // Command ID
} graphic_data_packhead; // Frame header

typedef __packed struct
{
   uint8_t figure_name[3];
   uint32_t operate_type : 3; // Add, Modify, Delete
   uint32_t figure_type : 3;
   uint32_t layer : 4;
   uint32_t color : 4;
   uint32_t details_a : 9;
   uint32_t details_b : 9;
   uint32_t width : 10;
   uint32_t start_x : 11;
   uint32_t start_y : 11;
   uint32_t details_c : 10;
   uint32_t details_d : 11;
   uint32_t details_e : 11;
} graphic_data_struct_t;

// Client Drawing Graphics
typedef __packed struct
{
   // TODO: Maybe try making this an array as supported by the protocol
   graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_t;

// Interactive Data Information
typedef __packed struct
{
   uint16_t data_cmd_id;                       // Data segment content ID
   uint16_t sender_ID;                         // Sender ID
   uint16_t receiver_ID;                       // Receiver ID
   ext_client_custom_graphic_t graphic_custom; // Custom graphic data
} ext_student_interactive_header_data_t;

typedef __packed struct
{
   graphic_data_struct_t graph_control;
   uint8_t show_data[30];
} string_data;

typedef __packed struct
{
   uint16_t data_cmd_id;      // Data segment content ID
   uint16_t sender_ID;        // Sender ID
   uint16_t receiver_ID;      // Receiver ID
   string_data string_custom; // Custom graphic data
} string_payload;

typedef __packed struct
{
   uint16_t data_cmd_ID; // Data segment content ID
   uint16_t sender_ID;   // Sender ID
   uint16_t receiver_ID; // Receiver ID
} graphic_data_head;     // => ext_student_interactive_header_data_t

typedef struct
{
   uint8_t delete_operate; // Delete operation
   uint8_t layer;          // Layer to be deleted
} graphic_delete;          // Frame for deleting a layer

void line_draw(graphic_data_struct_t *image, char figure_name[3], uint32_t graph_operate, uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);
void char_draw(string_data *image, char figure_name[3], uint32_t graph_operate, uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size, uint32_t graph_digit, uint32_t graph_width, uint32_t start_x, uint32_t start_y, char *char_data);
int update_ui(graphic_data_struct_t *image_ptr);
int update_char(string_data *string_ptr);
#endif
