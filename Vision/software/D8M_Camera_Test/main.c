/*
Author: LIM Tian Yi, Dr Ed Stott, Terasic Sample Code

Driver code for Nios II
*/

#include <stdio.h>
#include "I2C_core.h"

#define SPI_ONLY 1	    // Removes all camera-related code
#define KEY_DEBUG 1     // Remove after all coefficients are tuned for
#define UK_TEST 0       // Separate coefficients for SG and UK testing

#if SPI_ONLY
#include "terasic_includes.h"
#include "mipi_camera_config.h"
#include "mipi_bridge_config.h"
#include "auto_focus.h"
#endif

#include <fcntl.h>
#include <unistd.h>
// For SPI Slave
#include "altera_avalon_spi.h"
#include "altera_avalon_spi_regs.h"
#include "sys/alt_irq.h"

//EEE_IMGPROC defines
#define EEE_IMGPROC_MSG_START_RBB ('R'<<16 | 'B'<<8 | 'B')
#define EEE_IMGPROC_MSG_START_GBB ('G'<<16 | 'B'<<8 | 'B')
#define EEE_IMGPROC_MSG_START_BBB ('B'<<16 | 'B'<<8 | 'B')
#define EEE_IMGPROC_MSG_START_YBB ('Y'<<16 | 'B'<<8 | 'B')
#define EEE_IMGPROC_MSG_START_PBB ('P'<<16 | 'B'<<8 | 'B') // For the different colours of the rainbow
#define EEE_IMGPROC_MSG_START_WBB ('W'<<16 | 'B'<<8 | 'B')

//offsets
#define EEE_IMGPROC_STATUS 0
#define EEE_IMGPROC_MSG 1
#define EEE_IMGPROC_ID 2
#define EEE_IMGPROC_BBCOL 3
#define EEE_IMGPROC_THRESH 4    // tweaking thresholds
#define EEE_IMGPROC_MODE 5
alt_u8 disp_mode = 0;	        // boots into Colour Detection display

#define EXPOSURE_INIT 0x004000
#define EXPOSURE_STEP 0x100
#define GAIN_INIT 0x0800
#define GAIN_STEP 0x040
#define DEFAULT_LEVEL 3

#define MIPI_REG_PHYClkCtl		0x0056
#define MIPI_REG_PHYData0Ctl	0x0058
#define MIPI_REG_PHYData1Ctl	0x005A
#define MIPI_REG_PHYData2Ctl	0x005C
#define MIPI_REG_PHYData3Ctl	0x005E
#define MIPI_REG_PHYTimDly		0x0060
#define MIPI_REG_PHYSta			0x0062
#define MIPI_REG_CSIStatus		0x0064
#define MIPI_REG_CSIErrEn		0x0066
#define MIPI_REG_MDLSynErr		0x0068
#define MIPI_REG_FrmErrCnt		0x0080
#define MIPI_REG_MDLErrCnt		0x0090

alt_irq_context isr_context;

#define SPI_BUF_SIZE 13		// Leaving space for all 5 possible colours + wall
volatile int buf_index=0;	// buffer index for reading from context
volatile static alt_u8 spi_buf[SPI_BUF_SIZE];
volatile alt_u8 (*spi_buf_ptr)[SPI_BUF_SIZE] = &spi_buf;	// pointer to buffer
volatile alt_u8 datain;		// input data from SPI interrupt

#if SPI_ONLY
void mipi_clear_error(void){
	MipiBridgeRegWrite(MIPI_REG_CSIStatus,0x01FF); // clear error
	MipiBridgeRegWrite(MIPI_REG_MDLSynErr,0x0000); // clear error
	MipiBridgeRegWrite(MIPI_REG_FrmErrCnt,0x0000); // clear error
	MipiBridgeRegWrite(MIPI_REG_MDLErrCnt, 0x0000); // clear error

  	MipiBridgeRegWrite(0x0082,0x00);
  	MipiBridgeRegWrite(0x0084,0x00);
  	MipiBridgeRegWrite(0x0086,0x00);
  	MipiBridgeRegWrite(0x0088,0x00);
  	MipiBridgeRegWrite(0x008A,0x00);
  	MipiBridgeRegWrite(0x008C,0x00);
  	MipiBridgeRegWrite(0x008E,0x00);
  	MipiBridgeRegWrite(0x0090,0x00);
}

void mipi_show_error_info(void){
	alt_u16 PHY_status, SCI_status, MDLSynErr, FrmErrCnt, MDLErrCnt;

	PHY_status = MipiBridgeRegRead(MIPI_REG_PHYSta);
	SCI_status = MipiBridgeRegRead(MIPI_REG_CSIStatus);
	MDLSynErr = MipiBridgeRegRead(MIPI_REG_MDLSynErr);
	FrmErrCnt = MipiBridgeRegRead(MIPI_REG_FrmErrCnt);
	MDLErrCnt = MipiBridgeRegRead(MIPI_REG_MDLErrCnt);
	printf("PHY_status=%xh, CSI_status=%xh, MDLSynErr=%xh, FrmErrCnt=%xh, MDLErrCnt=%xh\r\n", PHY_status, SCI_status, MDLSynErr,FrmErrCnt, MDLErrCnt);
}

void mipi_show_error_info_more(void){
    printf("FrmErrCnt = %d\n",MipiBridgeRegRead(0x0080));
    printf("CRCErrCnt = %d\n",MipiBridgeRegRead(0x0082));
    printf("CorErrCnt = %d\n",MipiBridgeRegRead(0x0084));
    printf("HdrErrCnt = %d\n",MipiBridgeRegRead(0x0086));
    printf("EIDErrCnt = %d\n",MipiBridgeRegRead(0x0088));
    printf("CtlErrCnt = %d\n",MipiBridgeRegRead(0x008A));
    printf("SoTErrCnt = %d\n",MipiBridgeRegRead(0x008C));
    printf("SynErrCnt = %d\n",MipiBridgeRegRead(0x008E));
    printf("MDLErrCnt = %d\n",MipiBridgeRegRead(0x0090));
    printf("FIFOSTATUS = %d\n",MipiBridgeRegRead(0x00F8));
    printf("DataType = 0x%04x\n",MipiBridgeRegRead(0x006A));
    printf("CSIPktLen = %d\n",MipiBridgeRegRead(0x006E));
}

bool MIPI_Init(void){
	bool bSuccess;

	bSuccess = oc_i2c_init_ex(I2C_OPENCORES_MIPI_BASE, 50*1000*1000,400*1000); //I2C: 400K
	if (!bSuccess){
		printf("failed to init MIPI- Bridge i2c\r\n");
        while(1);   // exit
    }
    usleep(50*1000);
    MipiBridgeInit();

    usleep(500*1000);
    MipiCameraInit();

    MIPI_BIN_LEVEL(DEFAULT_LEVEL);

 	usleep(1000);
	return bSuccess;
}

alt_u16 leftRightToX(alt_u16 left, alt_u16 right){
	if (left==2047)	return 2047;
	return (left+right) / 2;
}

alt_u16 topBotToY(alt_u16 top, alt_u16 bottom){
	if (top==2047)	return 2047;
	return (top+bottom) / 2;
}

/* This is implemented on ESP32, as not enough prog mem to do this function.
#define pi 3.14152
#include <math.h>
alt_u8 wallDistFromRTheta(alt_u8 r, alt_u8 theta_in){
    int theta = theta_in*2 + 50;
    float dist = 0;  // Perform operations on this variable

    if (theta==90) return r;

    if (theta > 90) {
        theta -= 90;
        dist = (float)r * cos( (float)theta*pi/180 ) + (10.0 - (float)r * sin((float)theta*pi/180) )*(tan((float)theta*pi/180));
    } else {
        dist = (((float)r/cos((float)theta*pi/180))-10.0) * tan((float)(90-theta)*pi/180);
    }

    dist = 240 + dist*2;    // Y-value on screen

    if (dist > 435) return 33;
    if (dist < 265) return 254;

    int res = 7920/(dist-239);

    return res;
}
*/

// get image coordinates 
// Coord_vect: [red, green, blue, yellow, purple][yCoord, xCoord, left, right, top, bottom]
void IMGPROC_get_bb_coords(alt_16 coord_vect[6][6]){
    while ( (IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_STATUS)>>8 ) & 0xff) { 	//Find out if there are words to read
    	alt_u16 left, right, top, bottom, height, breadth;
    	float ratio;

        alt_u32 word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 			//Get next word from message buffer
        switch (word) {
            case EEE_IMGPROC_MSG_START_RBB: {	// Red
            	// printf("RBB:\n");
                word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
                coord_vect[0][3] = word & 0x7FF; 		    // right
                right = coord_vect[0][3];
                coord_vect[0][2] = (word >> 16) & 0x7FF;    // left
                left = coord_vect[0][2];

                word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
				coord_vect[0][5] = word & 0x7FF; 		    // bottom
				bottom = coord_vect[0][5];
				coord_vect[0][4] = (word >> 16) & 0x7FF;    //top
				top = coord_vect[0][4];

				height = bottom-top;
				breadth = right-left;

				ratio = height > breadth ? (float)height/(float)breadth : (float)breadth/(float)height;
//				printf("%d | %d | %f\n", height, breadth, ratio);
				if(ratio<1.25){
					coord_vect[0][1] = leftRightToX(left, right);
					coord_vect[0][0] = topBotToY(top, bottom);
				} else {
					coord_vect[0][1] = 2047;
					coord_vect[0][0] = 2047;
				}
                break;
            }
            case EEE_IMGPROC_MSG_START_GBB: {	// Green
            	// printf("GBB:\n");
                word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
                coord_vect[1][3] = word & 0x7FF;
                right = coord_vect[1][3]; 		    // left_green
                coord_vect[1][2] = (word >> 16) & 0x7FF;    // right_red
                left = coord_vect[1][2];

                word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
				coord_vect[1][5] = word & 0x7FF; 		    // left_red
                bottom = coord_vect[1][5];
				coord_vect[1][4] = (word >> 16) & 0x7FF;    // right_red
                top = coord_vect[1][4];

                height = bottom-top;
				breadth = right-left;

				ratio = height > breadth ? (float)height/(float)breadth : (float)breadth/(float)height;
				if(ratio<1.25){
					coord_vect[1][1] = leftRightToX(left, right);
					coord_vect[1][0] = topBotToY(top, bottom);
				} else {
					coord_vect[1][1] = 2047;
					coord_vect[1][0] = 2047;
				}
                break;
            }
            case EEE_IMGPROC_MSG_START_BBB: {	// Blue
            	// printf("BBB:\n");
                word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
                coord_vect[2][3] = word & 0x7FF;
                right = coord_vect[2][3]; 		    // left_red
                coord_vect[2][2] = (word >> 16) & 0x7FF;    // right_red
                left = coord_vect[2][2];

                word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
				coord_vect[2][5] = word & 0x7FF; 		    // left_red
                bottom = coord_vect[2][5];
				coord_vect[2][4] = (word >> 16) & 0x7FF;    // right_red
                top = coord_vect[2][4];

                height = bottom-top;
				breadth = right-left;

				ratio = height > breadth ? (float)height/(float)breadth : (float)breadth/(float)height;
				if(ratio<1.25){
					coord_vect[2][1] = leftRightToX(left, right);
					coord_vect[2][0] = topBotToY(top, bottom);
				} else {
					coord_vect[2][1] = 2047;
					coord_vect[2][0] = 2047;
				}
                break;
            }
            case EEE_IMGPROC_MSG_START_YBB: {	// Yellow
            	// printf("YBB:\n");
                word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
                coord_vect[3][3] = word & 0x7FF;
                right = coord_vect[3][3];    // left_red
                coord_vect[3][2] = (word >> 16) & 0x7FF;    // right_red
                left = coord_vect[3][2];

                word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
				coord_vect[3][5] = word & 0x7FF; 		    // left_red
                bottom = coord_vect[3][5];
				coord_vect[3][4] = (word >> 16) & 0x7FF;    // right_red
                top = coord_vect[3][4];

                height = bottom-top;
				breadth = right-left;

				ratio = height > breadth ? (float)height/(float)breadth : (float)breadth/(float)height;
				if(ratio<1.25){
					coord_vect[3][1] = leftRightToX(left, right);
					coord_vect[3][0] = topBotToY(top, bottom);
				} else {
					coord_vect[3][1] = 2047;
					coord_vect[3][0] = 2047;
				}
                break;
            }
            case EEE_IMGPROC_MSG_START_PBB: {	// Purple
            	// printf("PBB:\n");
                word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
                coord_vect[4][3] = word & 0x7FF;
                right = coord_vect[4][3]; 		    // left_red
                coord_vect[4][2] = (word >> 16) & 0x7FF;    // right_red
                left = coord_vect[4][2];

                word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
				coord_vect[4][5] = word & 0x7FF; 		    // left_red
                bottom = coord_vect[4][5];
				coord_vect[4][4] = (word >> 16) & 0x7FF;    // right_red
                top = coord_vect[4][4];

                height = bottom-top;
				breadth = right-left;

				ratio = height > breadth ? (float)height/(float)breadth : (float)breadth/(float)height;
				if(ratio<1.25){
					coord_vect[4][1] = leftRightToX(left, right);
					coord_vect[4][0] = topBotToY(top, bottom);
				} else {
					coord_vect[4][1] = 2047;
					coord_vect[4][0] = 2047;
				}
                break;
            }
            case EEE_IMGPROC_MSG_START_WBB: {   // Wall
            	// printf("WBB:\n");
            	word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
                coord_vect[5][1] = word & 0x7FF; 		    // Theta
                coord_vect[5][0] = (word >> 16) & 0x7FF;	// R
                break;
            }
            default:{
//                printf("!!%08x, Buf:%d\n", word, ((IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_STATUS)>>8 ) & 0xff));   // word that is not recognised
            }
        }
    }

    // Flush message buffer
    IOWR(EEE_IMGPROC_0_BASE, EEE_IMGPROC_STATUS, 0x10);
    IOWR(EEE_IMGPROC_0_BASE, EEE_IMGPROC_STATUS, 0x00);
}

void IMGPROC_set_thresh(alt_u16 addr, alt_u8 value){
    alt_u32 val = addr << 16 | value;
    printf("Wrote %08x\n", val);
    IOWR(EEE_IMGPROC_0_BASE, EEE_IMGPROC_THRESH, val);
}
#endif

// Distance mapping for balls
alt_u8 distFromYValue(alt_16 yVal){
    // yVal ranges 	from 0(top) to 479(bottom)
	if (yVal == -1) {return 255;}		// Invalid bounding box
    if (yVal > 435) { return 33; }	    // Minimum distance
    if (yVal < 265 ) { return 254; }    // upper bound, 255 is reserved invalid value
    return 6542/(yVal-239);	// Neat distance mapping in cm!
}

// Correction Enum: 00: None, 01: Right, 10: Left, 11: DC (one-hot for L/R)
alt_u8 leftRightFromXValue(alt_16 xVal){
    // x from 0-639. center 320. give it 20px of thresh.
    int center_thresh = 20;

    if (xVal < 320-center_thresh) return 0b10000000;  // item in the left of frame. turn left
    if (xVal > 320+center_thresh) return 0b01000000;  // item in the right of frame. turn right
    return 0b00000000;  // Nothing to return
}

struct color_state{
    alt_u8 pastCorr;
    alt_u8 edge;
    alt_u8 flag;
};

alt_16 coord_arr[6][6];	// x,y values for each distance
struct color_state color_states[5] = {
		{3,0,0}, {3,0,0}, {3,0,0}, {3,0,0}, {3,0,0}
};

// Updates the state struct to determine whether something happened
void updateState(struct color_state color_states[5]){
    for(int i=0; i<5; i++){
        if (coord_arr[i][0] != 2047) {  // Check for validity
            alt_u8 currCorr = (leftRightFromXValue(coord_arr[i][1]) >> 6) & 0b011;
            if (color_states[i].pastCorr==1 && currCorr==0){
                color_states[i].edge = 1;
            } else {
            	color_states[i].edge = 0;
            }
            color_states[i].pastCorr = currCorr;
        }
    }
}

void clearStateFlags(struct color_state color_states[5]){
    for(int i=0; i<5; i++){
    	color_states[i].pastCorr = 3;
    	if(color_states[i].flag != 0){
    		color_states[i].flag = 0;
    		printf("Flag %d cleared\n", i+1);
    	}
    }
}

// This has been edited to trigger whenever the colour is in the center.
// This should then work bi-directionally.
void writeGPIOfromState(struct color_state color_states[5], alt_u8 color){
    if (color==6){
        for(int i=0; i<5; i++){
            if (color_states[i].pastCorr==0 && color_states[i].flag==0){
                IOWR(GPIO_STOP_BASE, 0x0, 0b01);	// pull Drive low
                usleep(100);
                IOWR(GPIO_STOP_BASE, 0x0, 0b11);

            	alt_printf("\nSPI\n");
				for(int i=0; i<13; i++){
					printf("%d\n", spi_buf[i]);
				}
				alt_printf("\n");
				color_states[i].flag = 1;

            	printf("Cmd6 Col%d TRG\n", i+1);
            	for(int i=0; i<5; i++){
					printf("C_S[%d]:%d|%d|%d\n", i+1, color_states[i].pastCorr, color_states[i].edge, color_states[i].flag);
				}
            }
        }
    } else {
        if (color_states[color-1].pastCorr==1 && color_states[color-1].flag==0){

            IOWR(GPIO_STOP_BASE, 0x0, 0b01);	// bring Drive Low
            usleep(100);
            IOWR(GPIO_STOP_BASE, 0x0, 0b11);

            alt_printf("\nSPI\n");
			for(int i=0; i<13; i++){
				printf("%d\n", spi_buf[i]);
			}
			alt_printf("\n");
			color_states[color-1].flag = 1;	// Think about this cos it might need to be triggered multiple times
        	
            printf("Cmd%d Col%d TRG\n", color, color);
			printf("C_S[%d]:%d|%d|%d\n", color, color_states[color-1].pastCorr, color_states[color-1].edge, color_states[color-1].flag);
        }
    }
}

/* VALUES to output:
 * alt_u8 DISTANCE (8-bit)
 * alt_u8: 2MSBs--Correction, XXX, 3LSBs--COLOR
 *
 * Correction Enum: 00: None, 01: Right, 10: Left, 11: DC (one-hot for L/R)
 * Colour Enum:
 * 0: NONE/Invalid
 * 1: Red
 * 2: Green
 * 3: Blue
 * 4: Yellow
 * 5: Purple
 * 6: CHECK_SURROUNDINGS
 *
*/

// ISR for obtaining data from the FPGA
static void spi_rx_isr(void* isr_context){
	// Supposed to read everything from the buffer
	datain = IORD_ALTERA_AVALON_SPI_RXDATA(SPI_0_BASE);
	// Cast void pointer to array of alt_u8;
	alt_u8 (*buf_ptr)[SPI_BUF_SIZE] = isr_context;

	if(datain & 0x80){
        datain &= 0x7F; // get rid of the leading 1
		buf_index = 0;
	} else {
		buf_index++;
//		if((*buf_ptr)[buf_index] != 0xFF){ buf_index++; }	// transaction continues
//		else { buf_index = 12; }
	}   // Transaction always reads through entire message buffer regardless of data

	alt_u8 dataval = (*buf_ptr)[buf_index];

	// Don't forget operator precedence in doing stuff
	IOWR_ALTERA_AVALON_SPI_TXDATA(SPI_0_BASE, dataval);

//	alt_printf("!%x ", buf_index);	// Debugging

	//This resets the IRQ flag. Otherwise the IRQ will continuously run.
	IOWR_ALTERA_AVALON_SPI_STATUS(SPI_0_BASE, 0x0);
}

int main() {
#if SPI_ONLY
    IOWR(MIPI_PWDN_N_BASE, 0x00, 0x00);
    IOWR(MIPI_RESET_N_BASE, 0x00, 0x00);

    usleep(2000);
    IOWR(MIPI_PWDN_N_BASE, 0x00, 0xFF);
    usleep(2000);
    IOWR(MIPI_RESET_N_BASE, 0x00, 0xFF);
#endif
    int ret = alt_ic_isr_register(SPI_0_IRQ_INTERRUPT_CONTROLLER_ID, SPI_0_IRQ,
    								spi_rx_isr, (void*)spi_buf_ptr, 0x0);	// Set the ISR context to NULL...
    if (ret) {
    	alt_printf("SPI IRQ fail\n", ret);
    	while(1);
    }
    // Initialise buffer (last item always=255)
    spi_buf[SPI_BUF_SIZE-1] = 0xFF;

    IOWR(GPIO_STOP_BASE, 0x0, 0xF);	// High until STOP is called

    //You need to enable the IRQ in the IP core control register as well.
    IOWR_ALTERA_AVALON_SPI_CONTROL(SPI_0_BASE,
    			ALTERA_AVALON_SPI_CONTROL_SSO_MSK | ALTERA_AVALON_SPI_CONTROL_IRRDY_MSK);

#if SPI_ONLY
    // MIPI Init
    if ( !MIPI_Init()) {
        alt_printf("MIPI Init fail\n");
        while(1);
    }

    mipi_clear_error();
    usleep(50*1000);
    mipi_clear_error();
    usleep(1000*1000);
    mipi_show_error_info();
	    // mipi_show_error_info_more();

    // Continue to setup camera
//    alt_u16 bin_level = DEFAULT_LEVEL;
//    alt_u8  manual_focus_step = 10;
//    alt_u16  current_focus = 300;
//    int boundingBoxColour = 0
    alt_u32 exposureTime = EXPOSURE_INIT;
    alt_u16 gain = GAIN_INIT;

    OV8865SetExposure(exposureTime);
    OV8865SetGain(gain);
    Focus_Init();

    IOWR(EEE_IMGPROC_0_BASE, EEE_IMGPROC_MODE, disp_mode);

	//	Write default vals to IP core

	#if UK_TEST
	IMGPROC_set_thresh(2,21);		// r_h		255	if old HW
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(3,8);		// r_l		8	if old HW
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(4,128);		// g_h
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(5,77);		// g_l
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(6,166);		// b_h
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(7,123);		// b_l
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(8,57);		// y_h
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(9,40);		// y_l
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(10,240);		// p_h		25	if old HW
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(11,3);		// p_l		7	if old HW
	IMGPROC_set_thresh(0xFFFF, 0);

	IMGPROC_set_thresh(12,114);		// r_sat
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(13,63);		// r_val
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(14,70);		// green sat
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(15,25);		// green val
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(16,63);		// blue sat
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(17,25);		// blue val
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(18,127);		// yellow sat
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(19,102);		// yellow val
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(20,76);		// p_sat
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(21,65);		// p_val
	IMGPROC_set_thresh(0xFFFF, 0);

	#else
	IMGPROC_set_thresh(2,21);		// r_h		255	if old HW
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(3,8);		// r_l		8	if old HW
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(10,200);		// p_h		25	if old HW
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(11,0);		// p_l		7	if old HW
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(12,64);		// r_sat	76
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(13,63);		// r_val	63
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(20,50);		// p_sat
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(21,63);		// p_val	63
	IMGPROC_set_thresh(0xFFFF, 0);

	IMGPROC_set_thresh(14, 90);		// green sat
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(15, 38);		// green val
	IMGPROC_set_thresh(0xFFFF, 0);
//	IMGPROC_set_thresh(16, );		// blue val
//	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(17, 38);		// blue sat
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(18, 102);		// yellow sat
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(19, 76);		// yellow val
	IMGPROC_set_thresh(0xFFFF, 0);
	#endif

	IMGPROC_set_thresh(22,200);		// Size Threshold
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(23,1);		// Prox Threshold
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(24,15);		// Non-max Threshold
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(25, 10);		// Hough Count Threshold
	IMGPROC_set_thresh(0xFFFF, 0);
	IMGPROC_set_thresh(26, 30);		// Black level threshold
	IMGPROC_set_thresh(0xFFFF, 0);
#endif

	// Number of valid items in the SPI buffer
	alt_u8 numValid;
	alt_u16 ledWrite = 0;

    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    while(1){
        // Read from the IP core
        // Check if wall is valid. 1022 is less than -1 in a 11-bit (un)signed number.

//    	IOWR_ALTERA_AVALON_SPI_TXDATA(SPI_0_BASE, 0);	// Will read 0's for everything
//        alt_printf("SPI_Status:%x\n", IORD(SPI_0_BASE, 2));	// Read status of SPI buffer for debugging.

        IMGPROC_get_bb_coords(coord_arr);   // busy-polling for new data

        isr_context = alt_irq_disable_all();

        if (coord_arr[5][1] != 2047) {
        	// pack in the bits in a different way
            spi_buf[0] = coord_arr[5][0] & 0b01111111l;	// R (7-bit)
            spi_buf[1] = 0b00000110 | (coord_arr[5][1]<<3);	// Theta (5-bit) | Opcode=6
//        	spi_buf[0] = wallDistFromRTheta(coord_arr[5][0], coord_arr[5][1]);
//        	spi_buf[1] = 6;
            numValid = 1;
        } else {
            numValid = 0;
        }

        // LED switch (debugging)
        if (datain==0) {
        	ledWrite = 0x01;
        } else if (datain<=6) {
        	ledWrite = 1 | datain << 1;
        } else if (datain==7) {
        	ledWrite = 0;
        } else {
        	ledWrite = 0x02;
        }

        switch (datain) {
            // Needs some work.
            case 0: {
                numValid = 0;
                break;  // Nothing requested, nothing written.
            }
            case 1: {
                // Check if item is valid before stuffing into buffer
                if (coord_arr[0][1] != 2047) {
                    numValid += 1;
                    spi_buf[numValid*2+0] = distFromYValue(coord_arr[0][0]);
                    spi_buf[numValid*2+1] = (0b0001) | leftRightFromXValue(coord_arr[0][1]);
                }
                break;
            }
            case 2: {           // green
                if (coord_arr[1][1] != 2047) {
                    numValid += 1;
                    spi_buf[numValid*2+0] = distFromYValue(coord_arr[1][0]);
                    spi_buf[numValid*2+1] = (0b0010) | leftRightFromXValue(coord_arr[1][1]);
                }
                break;
            }
            case 3: {           // blue
                if (coord_arr[2][1] != 2047) {
                    numValid += 1;
                    spi_buf[numValid*2+0] = distFromYValue(coord_arr[2][0]);
                    spi_buf[numValid*2+1] = (0b0011) | leftRightFromXValue(coord_arr[2][1]);
                }
                break;
            }
            case 4: {           // yellow
                if (coord_arr[3][1] != 2047) {
                    numValid += 1;
                    spi_buf[numValid*2+0] = distFromYValue(coord_arr[3][0]);
                    spi_buf[numValid*2+1] = (0b0100) | leftRightFromXValue(coord_arr[3][1]);
                }
                break;
            }
            case 5: {           // purple
                if (coord_arr[4][1] != 2047) {
                    numValid += 1;
                    spi_buf[numValid*2+0] = distFromYValue(coord_arr[4][0]);
                    spi_buf[numValid*2+1] = (0b0101) | leftRightFromXValue(coord_arr[4][1]);
                }
                break;
            }
            case 6: {           // ALL
                for(int i=0; i<5; i++){
                    if(coord_arr[i][0] != 2047) {
                        spi_buf[2*numValid] = distFromYValue(coord_arr[i][0]);
                        spi_buf[2*numValid+1] = (i+1) | leftRightFromXValue(coord_arr[i][1]);
                        numValid++;
                    }
                }
                break;
            } 
            default: {
                numValid = 0;
            }
        }

        for(int i=numValid*2; i<SPI_BUF_SIZE; i++){
            // Overwrite invalid SPI_BUF values
            spi_buf[i] = 0xFF;
        }

        alt_irq_enable_all(isr_context);
        IOWR(LED_BASE, 0, 0x200 | ledWrite);

        // Stop if:
        // command is 6; we see some value in the center.
        // command is <color>; we see the color in the center.
        // datain=0 will be sent after the rotation to clear internal state.

        updateState(color_states);
        if (datain==0) clearStateFlags(color_states);
        writeGPIOfromState(color_states, datain);

        usleep(1000*15); // frame rate 60fps, so we can afford to delay to poll
        IOWR(LED_BASE, 0, ledWrite);

#if DEBUG
        // Key for triggering STOP interrupt
        if((IORD(KEY_BASE,0)&0x03) == 0x02){
        	if (switch_latch==0){
				printf("Interrupt called!\tnumValid=%d\n", numValid);
				for(int i=0; i<6; i++){
					printf("i %d:\n", i);
					for(int j=0; j<2; j++){
						printf("%04d ", coord_arr[i][j]);
					}
					alt_printf("\n");
				}

				IOWR(GPIO_STOP_BASE, 0x0, 0x0);
				usleep(1000);	// Delay to stop triggering
				IOWR(GPIO_STOP_BASE, 0x0, 0xF);
        	}
        	switch_latch = 1;	// Debounce
        } else {
        	switch_latch = 0;
        }
#endif

        //Process input commands from kb for tuning.
#if KEY_DEBUG
        int in = getchar();
        switch (in) {
#if SPI_ONLY
//        	IOWR_ALTERA_AVALON_SPI_TXDATA(SPI_0_BASE, 0);	// Will read 0's for everything
        	isr_context = alt_irq_disable_all();

       		case 'e': {
                exposureTime += EXPOSURE_STEP;
                OV8865SetExposure(exposureTime);
                alt_printf("\nExp: %x ", exposureTime);
                break;
            }
       	    case 'd': {
                exposureTime -= EXPOSURE_STEP;
                OV8865SetExposure(exposureTime);
                alt_printf("\nExp: %x ", exposureTime);
                break;
            }
       	    case 't': {
                gain += GAIN_STEP;
                OV8865SetGain(gain);
                alt_printf("\nGain: %x ", gain);
                break;
            }
       	    case 'g': {
                gain -= GAIN_STEP;
                OV8865SetGain(gain);
                alt_printf("\nGain: %x ", gain);
                break;
            }
       	    case 'p':{
       	    	// read output
       	        IMGPROC_get_bb_coords(coord_arr);
       	        for(int i=0; i<5; i++){
       	            alt_printf("i %x:\n", i);
       	            for(int j=0; j<6; j++){
       	                printf("%03d ", coord_arr[i][j]);
       	            }
       	            alt_printf("\n");
       	        }
       	        alt_printf("i 6:\n");
       	        printf("%03d %03d\n\n", coord_arr[5][0], (coord_arr[5][1]==2047) ? 2047 : coord_arr[5][1]*2+50);

       	        alt_printf("\nSPI Buf(%x):\n", numValid);
            	for(int i=0; i<SPI_BUF_SIZE; i++){
            		printf("%d\n", spi_buf[i]);
            	}
            	alt_printf("\n");

       	    	break;
       	    }
            case 'x': {
            	// Set to blocking for user input
				fcntl(STDIN_FILENO, F_SETFL, ~O_NONBLOCK);

                char addr_buf[] = "00";
                char val_buf[] = "000";
                int addr, val;

                alt_u8 pass = 0;
                alt_u8 skip = 0;
                
                while(!pass){
                    alt_printf("Input adr(2-digit):\n");
                    // Get modified thresholds
                    addr_buf[0] = getchar();
                    addr_buf[1] = getchar();    // adr
                    addr = atoi(addr_buf);
                    printf("Adr %02d\n", addr);
                    if (addr_buf[0]=='x' || addr_buf[1]=='x'){
                    	alt_printf("\'x\' pressed. Exit\n");
                    	skip = 1;
                    	break;
                    }
                    if (addr > 26){
                        pass = 0;
                        alt_printf("Invalid reg <= 25.\n");
                    } else {
                        pass = 1;
                    }
                }

                pass = 0;
                while (!pass && !skip) {
                    alt_printf("Input val(3-digit):\n");
                    val_buf[0] = getchar();
                    val_buf[1] = getchar();
                    val_buf[2] = getchar();
                    val = atoi(val_buf);
                    printf("Val: %03d\n", val);
                    if (val_buf[0]=='x' || val_buf[1]=='x' || val_buf[2]=='x'){
                    	skip = 1;
                    	alt_printf("\'x\' pressed. Exit\n");
                    	break;
                    }
                    if(addr < 2 || ( addr >= 12 && addr <= 21 )){
                    	// Val and Sat setting
                        if (val > 100) {
                            alt_printf("Invalid val <= 100.\n");
                            pass = 0;
                        } else {
                            pass = 1;
                            val = val * 256 / 100;
                        }
                    } else if (addr >= 2 && addr <=11) {
                    	// Colours (hue setting)
                        if (val > 360) {
                            alt_printf("Invalid val <= 360.\n");
                            pass = 0;
                        } else {
                            pass = 1;
                            val = val *256 / 360;
                        }
                    } else {
                    	// leave val unchanged
                    	pass = 1;
                    }
                }

                if (!skip) {
					printf("Val write: %03d\n", val);
					IMGPROC_set_thresh(addr, val);
					IMGPROC_set_thresh(0xFFFF, 0);	// end of writing
                }
                
                fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
                break;
            }
            case 'm':{
            	disp_mode = (disp_mode==3) ? 0 : disp_mode + 1;
            	alt_printf("Mode:%x\n", disp_mode);
				IOWR(EEE_IMGPROC_0_BASE, EEE_IMGPROC_MODE, disp_mode);
				break;
			}
#endif
            alt_irq_enable_all(isr_context);
		}
#endif

    }
    return 0;
}
