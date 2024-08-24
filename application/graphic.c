// TODO: Test to see if Circle_Draw() works
// TODO: Test to see if UI_Delete() works
// TODO: Implement more shapes to draw

#include "graphic.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
// #include "referee.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern uint8_t CRC8_INIT;
extern uint16_t CRC16_INIT;
uint8_t UI_Seq; // Packet sequence number

uint16_t get_receiver_id(uint16_t sender_ID)
{
	switch (sender_ID)
	{
		case UI_Data_RobotID_RHero:
			return UI_Data_CilentID_RHero; // For hero operator client (red)
		case UI_Data_RobotID_REngineer:
			return UI_Data_CilentID_REngineer; // For engineer operator client (red)
		case UI_Data_RobotID_RStandard1:
			return UI_Data_CilentID_RStandard1; // For standard 1 operator client (red)
		case UI_Data_RobotID_RStandard2:
			return UI_Data_CilentID_RStandard2; // For standard 2 operator client (red)
		case UI_Data_RobotID_RStandard3:
			return UI_Data_CilentID_RStandard3; // For standard 3 operator client (red)
		case UI_Data_RobotID_RAerial:
			return UI_Data_CilentID_RAerial; // For aerial operator client (red)
		case UI_Data_RobotID_BHero:
			return UI_Data_CilentID_BHero; // For hero operator client (blue)
		case UI_Data_RobotID_BEngineer:
			return UI_Data_CilentID_BEngineer; // For engineer operator client (blue)
		case UI_Data_RobotID_BStandard1:
			return UI_Data_CilentID_BStandard1; // For standard 1 operator client (blue)
		case UI_Data_RobotID_BStandard2:
			return UI_Data_CilentID_BStandard2; // For standard 2 operator client (blue)
		case UI_Data_RobotID_BStandard3:
			return UI_Data_CilentID_BStandard3; // For standard 3 operator client (blue)
		case UI_Data_RobotID_BAerial:
			return UI_Data_CilentID_BAerial; // For aerial operator client (blue)
		default:
			return UI_Data_CilentID_BStandard1;
	}
}

void ui_sendbyte(unsigned char ch)
{
	HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 999);
}

/********************************************Delete Operation*************************************
**Parameters: Del_Operate  Corresponding header file deletion operation
              Del_Layer    Layer to be deleted, values from 0 to 9
*****************************************************************************************/

void ui_delete(uint8_t del_operate, uint8_t del_layer)
{
	unsigned char *framepoint;       // Read-write pointer
	uint16_t frametail = CRC16_INIT; // CRC16 checksum value
	int loop_control;                // For loop control variable

	graphic_data_packhead framehead; // General Frame Header
	graphic_data_head datahead;      // "Sub-header" for graphics
	graphic_delete del;              // Delete operation

	framepoint = (unsigned char *)&framehead;

	framehead.SOF = UI_SOF;
	framehead.data_length = 8;
	framehead.seq = UI_Seq;
	framehead.CRC8 = get_CRC8_check_sum(framepoint, 4, CRC8_INIT);
	framehead.cmd_ID = STUDENT_INTERACTIVE_DATA_CMD_ID; // Fill in the header data

	datahead.data_cmd_ID = UI_Data_ID_Del;
	/*SZL 6-16-2022 Dynamically receive the referee system ID*/
	// TODO: Make sure this works
	// datahead.sender_ID = get_robot_id(); // Robot_ID;
	datahead.sender_ID = UI_Data_RobotID_BStandard3; // Robot_ID;

	switch (datahead.sender_ID)
	{
		case UI_Data_RobotID_RHero:
			datahead.receiver_ID = UI_Data_CilentID_RHero; // For hero operator client (red)
			break;
		case UI_Data_RobotID_REngineer:
			datahead.receiver_ID = UI_Data_CilentID_REngineer; // For engineer operator client (red)
			break;
		case UI_Data_RobotID_RStandard1:
			datahead.receiver_ID = UI_Data_CilentID_RStandard1; // For standard 1 operator client (red)
			break;
		case UI_Data_RobotID_RStandard2:
			datahead.receiver_ID = UI_Data_CilentID_RStandard2; // For standard 2 operator client (red)
			break;
		case UI_Data_RobotID_RStandard3:
			datahead.receiver_ID = UI_Data_CilentID_RStandard3; // For standard 3 operator client (red)
			break;
		case UI_Data_RobotID_RAerial:
			datahead.receiver_ID = UI_Data_CilentID_RAerial; // For aerial operator client (red)
			break;
		case UI_Data_RobotID_BHero:
			datahead.receiver_ID = UI_Data_CilentID_BHero; // For hero operator client (blue)
			break;
		case UI_Data_RobotID_BEngineer:
			datahead.receiver_ID = UI_Data_CilentID_BEngineer; // For engineer operator client (blue)
			break;
		case UI_Data_RobotID_BStandard1:
			datahead.receiver_ID = UI_Data_CilentID_BStandard1; // For standard 1 operator client (blue)
			break;
		case UI_Data_RobotID_BStandard2:
			datahead.receiver_ID = UI_Data_CilentID_BStandard2; // For standard 2 operator client (blue)
			break;
		case UI_Data_RobotID_BStandard3:
			datahead.receiver_ID = UI_Data_CilentID_BStandard3; // For standard 3 operator client (blue)
			break;
		case UI_Data_RobotID_BAerial:
			datahead.receiver_ID = UI_Data_CilentID_BAerial; // For aerial operator client (blue)
			break;
		default:
			// Do nothing
			return;
	}

	// Fill in operation data
	del.delete_operate = del_operate;
	del.layer = del_layer; // Control information

	// CRC16 checksum calculation
	frametail = get_CRC16_check_sum(framepoint, sizeof(framehead), frametail);
	framepoint = (unsigned char *)&datahead;
	frametail = get_CRC16_check_sum(framepoint, sizeof(datahead), frametail);
	framepoint = (unsigned char *)&del;
	frametail = get_CRC16_check_sum(framepoint, sizeof(del), frametail);

	// Send all frames
	framepoint = (unsigned char *)&framehead;
	for (loop_control = 0; loop_control < sizeof(framehead); loop_control++)
	{
		ui_sendbyte(*framepoint);
		framepoint++;
	}
	framepoint = (unsigned char *)&datahead;
	for (loop_control = 0; loop_control < sizeof(datahead); loop_control++)
	{
		ui_sendbyte(*framepoint);
		framepoint++;
	}
	framepoint = (unsigned char *)&del;
	for (loop_control = 0; loop_control < sizeof(del); loop_control++)
	{
		ui_sendbyte(*framepoint);
		framepoint++;
	}

	// Send CRC16 checksum value
	framepoint = (unsigned char *)&frametail;
	for (loop_control = 0; loop_control < sizeof(frametail); loop_control++)
	{
		ui_sendbyte(*framepoint);
		framepoint++;
	}

	// Increment sequence number
	UI_Seq++;
}

/**
 * @brief          Draw a straight line
 * @param[in]      *image: a pointer to a Graph_Data variable for storing graphic data
 * @param[in]      figure_name[3]: the image name used for identification
 * @param[in]      graph_operate: image operation, as defined in the header file (add, modify or delete)
 * @param[in]      graph_layer: layer index from 0 to 9
 * @param[in]      graph_color: color of the graphic
 * @param[in]      graph_width: line width of the graphic
 * @param[in]      start_x: starting x-coordinate
 * @param[in]      start_y: starting y-coordinate
 * @param[in]      end_x: ending x-coordinate
 * @param[in]      end_y: ending y-coordinate
 */
void line_draw(graphic_data_struct_t *image, char figure_name[3], uint32_t graph_operate, uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y)
{
	int i;
	for (i = 0; i < 3 && figure_name[i] != '\0'; i++)
		image->figure_name[i] = figure_name[i];

	image->operate_type = graph_operate;
	image->figure_type = UI_Graph_Line;
	image->layer = graph_layer;
	image->color = graph_color;
	image->details_a = 0;
	image->details_b = 0;
	image->width = graph_width;
	image->start_x = start_x;
	image->start_y = start_y;
	// image->details_c = 0;
	image->details_d = end_x;
	image->details_e = end_y;
}

/**
 * @brief          Draw a rectangle
 * @param[in]      *image: Pointer to a Graph_Data type variable for storing graphic data
 * @param[in]      figure_name[3]: Image name used for identification
 * @param[in]      graph_operate: Image operation, as defined in the header file
 * @param[in]      graph_layer: Graph layer (0-9)
 * @param[in]      graph_color: Graphic color
 * @param[in]      graph_width: Graphic line width
 * @param[in]      start_x: x-coordinate of the starting point
 * @param[in]      start_y: y-coordinate of the starting point
 * @param[in]      end_x: x-coordinate of the ending point (coordinates of the opposite corner)
 * @param[in]      end_y: y-coordinate of the ending point (coordinates of the opposite corner)
 */
void rectangle_draw(graphic_data_struct_t *image, char figure_name[3], uint32_t graph_operate, uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y)
{
	int i;
	for (i = 0; i < 3 && figure_name[i] != '\0'; i++)
		image->figure_name[i] = figure_name[i];

	image->figure_type = UI_Graph_Rectangle;
	image->operate_type = graph_operate;
	image->layer = graph_layer;
	image->color = graph_color;
	image->width = graph_width;
	image->start_x = start_x;
	image->start_y = start_y;
	image->details_d = end_x;
	image->details_e = end_y;
}

/**
 * @brief          Draw a complete circle
 * @param[in]      *image: a pointer to a Graph_Data variable for storing graphic data
 * @param[in]      figure_name[3]: the image name used for identification
 * @param[in]      graph_operate: image operation, as defined in the header file
 * @param[in]      graph_layer: layer index from 0 to 9
 * @param[in]      graph_color: color of the graphic
 * @param[in]      graph_width: line width of the graphic
 * @param[in]      start_x: x-coordinate of the center of the circle
 * @param[in]      start_y: y-coordinate of the center of the circle
 * @param[in]      graph_radius: radius of the graphic
 */
void circle_draw(graphic_data_struct_t *image, char figure_name[3], uint32_t graph_operate, uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width, uint32_t start_x, uint32_t start_y, uint32_t graph_radius)
{
	for (int i = 0; i < 3 && figure_name[i] != '\0'; i++)
		image->figure_name[i] = figure_name[i];

	image->figure_type = UI_Graph_Circle;
	image->operate_type = graph_operate;
	image->layer = graph_layer;
	image->color = graph_color;
	image->width = graph_width;
	image->start_x = start_x;
	image->start_y = start_y;
	image->details_c = graph_radius; // Radius of circle
}

/**
 * @brief          Draw an arc
 * @param[in]      *image: a pointer to a Graph_Data variable for storing graphic data
 * @param[in]      figure_name[3]: the image name used for identification
 * @param[in]      graph_operate: image operation, as defined in the header file
 * @param[in]      graph_layer: layer index from 0 to 9
 * @param[in]      graph_color: color of the graphic
 * @param[in]      graph_startangle: starting angle of the arc
 * @param[in]      graph_endangle: ending angle of the arc
 * @param[in]      graph_width: line width of the graphic
 * @param[in]      start_x: starting x-coordinate
 * @param[in]      start_y: starting y-coordinate
 * @param[in]      x_length: length along the x-axis
 * @param[in]      y_length: length along the y-axis
 */
void arc_draw(graphic_data_struct_t *image, char figure_name[3], uint32_t graph_operate, uint32_t graph_layer, uint32_t graph_color, uint32_t graph_startangle, uint32_t graph_endangle, uint32_t graph_width, uint32_t start_x, uint32_t start_y, uint32_t x_length, uint32_t y_length)
{
	int i;

	for (i = 0; i < 3 && figure_name[i] != '\0'; i++)
		image->figure_name[i] = figure_name[i];

	image->figure_type = UI_Graph_Arc;
	image->operate_type = graph_operate;
	image->layer = graph_layer;
	image->color = graph_color;
	image->width = graph_width;
	image->start_x = start_x;
	image->start_y = start_y;
	image->details_a = graph_startangle;
	image->details_b = graph_endangle;
	image->details_d = x_length;
	image->details_e = y_length;
}

/**
 * @brief          Draw floating-point data
 * @param[in]      *image: a pointer to a Graph_Data variable for storing graphic data
 * @param[in]      imagename[3]: the image name used for identification
 * @param[in]      graph_operate: image operation, as defined in the header file
 * @param[in]      graph_layer: layer index from 0 to 9
 * @param[in]      graph_color: color of the graphic
 * @param[in]      graph_width: line width of the graphic
 * @param[in]      graph_size: font size
 * @param[in]      start_x: starting x-coordinate
 * @param[in]      start_y: starting y-coordinate
 * @param[in]      graph_float: variable to display, the value gets divided by 1000 and then displayed
 *                              ex. if graph_float = 1234, the display will show 1.234
 */
// void float_draw(graphic_data_struct_t *image, char imagename[3], uint32_t graph_operate, uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size, uint32_t graph_digit, uint32_t graph_width, uint32_t start_x, uint32_t start_y, uint32_t graph_float)
// {
//     int i;
//     for (i = 0; i < 3 && imagename[i] != '\0'; i++)
//         image->figure_name[i] = imagename[i];

//     image->figure_type = UI_Graph_Float;
//     image->operate_type = graph_operate;
//     image->layer = graph_layer;
//     image->color = graph_color;
//     image->width = graph_width;
//     image->start_x = start_x;
//     image->start_y = start_y;
//     image->details_a = graph_size;

//     // Get the last 11 bits
//     unsigned mask = (1 << 11) - 1;
//     image -> details_e = graph_float & mask;

//     // Get the next 11 bits
//     mask = mask << 11;
//     image -> details_d = graph_float & mask;

//     // Get the first 10 bits
//     mask = ((1 << 10) - 1) << 22;
//     image->details_c = graph_float & mask;
// }

/**
 * @brief          Draw character data
 * @param[in]      *image: a pointer to a Graph_Data variable for storing graphic data
 * @param[in]      figure_name[3]: the image name used for identification
 * @param[in]      graph_operate: image operation, as defined in the header file
 * @param[in]      graph_layer: layer index from 0 to 9
 * @param[in]      graph_color: color of the graphic
 * @param[in]      graph_size: font size
 * @param[in]      graph_digit: number of characters
 * @param[in]      graph_width: line width of the graphic
 * @param[in]      start_x: starting x-coordinate
 * @param[in]      start_y: starting y-coordinate
 * @param[in]      *char_data: pointer to the start of the string data to be drawn
 */
// void char_draw(string_data *image, char figure_name[3], uint32_t graph_operate, uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size, uint32_t graph_digit, uint32_t graph_width, uint32_t start_x, uint32_t start_y, char *char_data)
// {
//     int i;

//     for (i = 0; i < 3 && figure_name[i] != '\0'; i++)
//         image->graph_control.figure_name[i] = figure_name[i];

//     memset(image->show_data, 0, 30);
//     for (i = 0; i < graph_digit; i++)
//     {
//         image->show_data[i] = *char_data;
//         char_data++;
//     }

//     image->graph_control.figure_type = UI_Graph_Char;
//     image->graph_control.operate_type = graph_operate;
//     image->graph_control.layer = graph_layer;
//     image->graph_control.color = graph_color;
//     image->graph_control.width = graph_width;
//     image->graph_control.start_x = start_x;
//     image->graph_control.start_y = start_y;
//     image->graph_control.details_a = graph_size;
//     image->graph_control.details_b = graph_digit;

// //    int i;

// //    for(i=0;i<3&&imagename[i]!=\0;i++)
// //       image->graph_control.graphic_name[2-i]=imagename[i];
// //    image->Graph_Control.graphic_tpye = UI_Graph_Char;
// //    image->Graph_Control.operate_tpye = Graph_Operate;
// //    image->Graph_Control.layer = Graph_Layer;
// //    image->Graph_Control.color = Graph_Color;
// //    image->Graph_Control.width = Graph_Width;
// //    image->Graph_Control.start_x = Start_x;
// //    image->Graph_Control.start_y = Start_y;
// //    image->Graph_Control.start_angle = Graph_Size;
// //    image->Graph_Control.end_angle = Graph_Digit;

// //    for(i=0;i<Graph_Digit;i++)
// //    {
// //       image->show_Data[i]=*Char_Data;
// //       Char_Data++;
// //    }
// }

void char_draw(string_data *image, char figure_name[3], uint32_t graph_operate, uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size, uint32_t graph_digit, uint32_t graph_width, uint32_t start_x, uint32_t start_y, char *char_data)
{
	int i;

	for (i = 0; i < 3 && figure_name[i] != NULL; i++)
		image->graph_control.figure_name[2 - i] = figure_name[i];

	// memset(image->show_data, 0, 30);
	for (i = 0; i < graph_digit; i++)
	{
		image->show_data[i] = *char_data;
		char_data++;
	}
	image->graph_control.figure_type = UI_Graph_Char;
	image->graph_control.operate_type = graph_operate;
	image->graph_control.layer = graph_layer;
	image->graph_control.color = graph_color;
	image->graph_control.width = graph_width;
	image->graph_control.start_x = start_x;
	image->graph_control.start_y = start_y;
	image->graph_control.details_a = graph_size;
	image->graph_control.details_b = graph_digit;
}

void float_draw(string_data *image, char figure_name[3], uint32_t graph_operate, uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size, uint32_t graph_digit, uint32_t graph_width, uint32_t start_x, uint32_t start_y, float graph_float)
{
	int i;
	char float_string[30]; // Buffer to hold the float as a string

	// Convert float to string with specified precision
	snprintf(float_string, sizeof(float_string), "%.*f", graph_digit, graph_float);

	// Copy figure_name to image->graph_control.figure_name
	for (i = 0; i < 3 && figure_name[i] != '\0'; i++)
		image->graph_control.figure_name[i] = figure_name[i];

	// Fill in the rest of the image->graph_control fields
	image->graph_control.figure_type = UI_Graph_Char;
	image->graph_control.operate_type = graph_operate;
	image->graph_control.layer = graph_layer;
	image->graph_control.color = graph_color;
	image->graph_control.width = graph_width;
	image->graph_control.start_x = start_x;
	image->graph_control.start_y = start_y;
	image->graph_control.details_a = graph_size;
	image->graph_control.details_b = graph_digit;

	// Clear the show_data array
	memset(image->show_data, 0, 30);

	// Copy the float string to image->show_data
	for (i = 0; i < graph_digit && float_string[i] != '\0'; i++)
	{
		image->show_data[i] = float_string[i];
	}
}

void referee_data_pack_handle(uint8_t sof, uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
	uint8_t tx_buff[MAX_SIZE];

	uint16_t frame_length = FRAMEHEADER_LEN + CMD_LEN + len + CRC_LEN; // Data frame length

	memset(tx_buff, 0, frame_length); // Clear the array for storing data

	/***** Frame Header Packing *****/
	tx_buff[0] = sof;                                  // Start byte of the data frame
	memcpy(&tx_buff[1], (uint8_t *)&len, sizeof(len)); // Length of the data in the data frame
	tx_buff[3] = UI_Seq;                               // Packet sequence number
	append_CRC8_check_sum(tx_buff, FRAMEHEADER_LEN);   // Frame header CRC8 checksum

	/***** Command Code Packing *****/
	memcpy(&tx_buff[FRAMEHEADER_LEN], (uint8_t *)&cmd_id, CMD_LEN);

	/***** Data Packing *****/
	memcpy(&tx_buff[FRAMEHEADER_LEN + CMD_LEN], p_data, len);
	append_CRC16_check_sum(tx_buff, frame_length); // Data frame CRC16 checksum

	if (UI_Seq == 0xff)
		UI_Seq = 0;
	else
		UI_Seq++;

	for (int i = 0; i < frame_length; i++)
	{
		HAL_UART_Transmit(&huart6, tx_buff + i, 1, 99);
	}
}

// TODO: Maybe make it so can upload multiple graphics at once as supported by the protocol
int update_ui(graphic_data_struct_t *image_ptr)
{
	ext_student_interactive_header_data_t custom_grapic_draw;

	uint16_t sender_id = 1;
	uint16_t receiver_id = get_receiver_id(sender_id);

	custom_grapic_draw.data_cmd_id = 0x0101; // Draw one graphics (Content ID, refer to the referee system manual for queries)

	custom_grapic_draw.sender_ID = sender_id;     // Sender ID, corresponding to the robot ID, in this case, the Blue Standard
	custom_grapic_draw.receiver_ID = receiver_id; // Receiver ID, operator client ID, in this case, the Blue Standard operator client

	memcpy(&custom_grapic_draw.graphic_custom.grapic_data_struct, image_ptr, sizeof(graphic_data_struct_t));

	referee_data_pack_handle(UI_SOF, UI_CMD_Robo_Exchange, (uint8_t *)&custom_grapic_draw, sizeof(custom_grapic_draw));

	return 0;
}

int update_char(string_data *string_ptr)
{
	string_payload custom_grapic_draw;

	uint16_t sender_id = 1;
	uint16_t receiver_id = get_receiver_id(sender_id);

	custom_grapic_draw.data_cmd_id = UI_Data_ID_DrawChar; // Draw one graphics (Content ID, refer to the referee system manual for queries)

	custom_grapic_draw.sender_ID = sender_id;     // Sender ID, corresponding to the robot ID, in this case, the Blue Standard
	custom_grapic_draw.receiver_ID = receiver_id; // Receiver ID, operator client ID, in this case, the Blue Standard operator client

	// memcpy(&custom_grapic_draw.graphic_custom.grapic_data_struct, string_ptr, sizeof(graphic_data_struct_t));
	memcpy(&custom_grapic_draw.string_custom, string_ptr, sizeof(string_data));

	referee_data_pack_handle(UI_SOF, UI_CMD_Robo_Exchange, (uint8_t *)&custom_grapic_draw, 45);

	return 0;
}
