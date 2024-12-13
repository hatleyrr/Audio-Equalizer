//This module was originally created by Dr. Peter Jamieson, and was modified in indicated places by Royce Hatley and Samuel Kanaan.
module DE1_SoC_Audio_Example (

    	//////////// ADC //////////
	//output		          		ADC_CONVST,
	//output		          		ADC_DIN,
	//input 		          		ADC_DOUT,
	//output		          		ADC_SCLK,

	//////////// Audio //////////
	input 		          		AUD_ADCDAT,
	inout 		          		AUD_ADCLRCK,
	inout 		          		AUD_BCLK,
	output		          		AUD_DACDAT,
	inout 		          		AUD_DACLRCK,
	output		          		AUD_XCK,

	//////////// CLOCK //////////
	//input 		          		CLOCK2_50,
	//input 		          		CLOCK3_50,
	//input 		          		CLOCK4_50,
	input 		          		CLOCK_50,

	//////////// SDRAM //////////
	//output		    [12:0]		DRAM_ADDR,
	//output		     [1:0]		DRAM_BA,
	//output		          		DRAM_CAS_N,
	//output		          		DRAM_CKE,
	//output		          		DRAM_CLK,
	//output		          		DRAM_CS_N,
	//inout 		    [15:0]		DRAM_DQ,
	//output		          		DRAM_LDQM,
	//output		          		DRAM_RAS_N,
	//output		          		DRAM_UDQM,
	//output		          		DRAM_WE_N,

	//////////// I2C for Audio and Video-In //////////
	output		          		FPGA_I2C_SCLK,
	inout 		          		FPGA_I2C_SDAT,

	//////////// SEG7 //////////
	output		     [6:0]		HEX0,
	output		     [6:0]		HEX1,
	output		     [6:0]		HEX2,
	output		     [6:0]		HEX3,
	output		     [6:0]		HEX4,
	output		     [6:0]		HEX5,

	//////////// IR //////////
	//input 		          		IRDA_RXD,
	//output		          		IRDA_TXD,

	//////////// KEY //////////
	input 		     [3:0]		KEY,

	//////////// LED //////////
	output		     [9:0]		LEDR,

	//////////// PS2 //////////
	//inout 		          		PS2_CLK,
	//inout 		          		PS2_CLK2,
	//inout 		          		PS2_DAT,
	//inout 		          		PS2_DAT2,


	//////////// Video-In //////////
	//input 		          		TD_CLK27,
	//input 		     [7:0]		TD_DATA,
	//input 		          		TD_HS,
	//output		          		TD_RESET_N,
	//input 		          		TD_VS,

	//////////// VGA //////////
	//output		          		VGA_BLANK_N,
	//output		     [7:0]		VGA_B,
	//output		          		VGA_CLK,
	//output		     [7:0]		VGA_G,
	//output		          		VGA_HS,
	//output		     [7:0]		VGA_R,
	//output		          		VGA_SYNC_N,
	//output		          		VGA_VS,

	//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
	//inout 		    [35:0]		GPIO_0,

	//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
	//inout 		    [35:0]		GPIO_1

	//////////// SW //////////
	input 		     [9:0]		SW
);

// Displays adjusted with frequency band names - Modified by Royce Hatley and Samuel Kanaan.
assign	HEX0		=	7'd1111001; //I
assign	HEX1		=	7'd0001001; //H
assign	HEX2		=	7'd0111111; //-
assign	HEX3		=	7'd0111111; //-
assign	HEX4		=	7'b1000000; //O
assign	HEX5		=	7'b1000111; //L


// DONE STANDARD PORT DECLARATION ABOVE
/* HANDLE SIGNALS FOR CIRCUIT */

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/


/*****************************************************************************
 *                 Internal Wires and Registers Declarations                 *
 *****************************************************************************/
wire clk;
assign clk = CLOCK_50;
wire rst;
assign rst = KEY[0];
// Add wire for equalizer start button, which will become "start" in the filter module - Added by Royce Hatley and Samuel Kanaan
wire equalize;
assign equalize = ~KEY[3];
 
// Internal Wires
wire				audio_in_available;
wire		[15:0]	left_channel_audio_in;
wire		[15:0]	right_channel_audio_in;

wire				audio_out_allowed;
wire		[15:0]	left_channel_audio_out;
wire		[15:0]	right_channel_audio_out;
//assign write_audio_out			= audio_in_available & audio_out_allowed;
//assign read_audio_in			= audio_in_available & audio_out_allowed;
reg write_audio_out;
reg read_audio_in;

// Internal Registers


// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                            Combinational Logic                            *
 *****************************************************************************/



// WHEN 1 record and play ELSE pass through
//assign left_channel_audio_out	= SW[0] ? ((playing == 1'b1) ? left_sample_q : 16'd0) : left_channel_audio_in;
assign left_channel_audio_out	= SW[0] ? ((playing == 1'b1) ? last_sample_q[31:16] : 16'd0) : left_channel_audio_in;
//assign right_channel_audio_out	= SW[0] ? ((playing == 1'b1) ? right_sample_q  : 16'd0) : right_channel_audio_in;
assign right_channel_audio_out = SW[0] ? ((playing == 1'b1) ? last_sample_q[15:0] : 16'd0) : right_channel_audio_in;



/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Audio_Controller Audio_Controllers (
	// Inputs
	.CLOCK_50					(clk),
	.reset						(~rst),

	.clear_audio_in_memory		(),
	.read_audio_in				(read_audio_in),
	
	.clear_audio_out_memory		(),
	.left_channel_audio_out		(left_channel_audio_out),
	.right_channel_audio_out	(right_channel_audio_out),
	.write_audio_out			(write_audio_out),

	.AUD_ADCDAT					(AUD_ADCDAT),

	// Bidirectionals
	.AUD_BCLK					(AUD_BCLK),
	.AUD_ADCLRCK				(AUD_ADCLRCK),
	.AUD_DACLRCK				(AUD_DACLRCK),

	// Outputs
	.audio_in_available			(audio_in_available),
	.left_channel_audio_in		(left_channel_audio_in),
	.right_channel_audio_in		(right_channel_audio_in),

	.audio_out_allowed			(audio_out_allowed),

	.AUD_XCK					(AUD_XCK),
	.AUD_DACDAT					(AUD_DACDAT)
);

avconf #(.USE_MIC_INPUT(1)) avc (
	.FPGA_I2C_SCLK				(FPGA_I2C_SCLK),
	.FPGA_I2C_SDAT				(FPGA_I2C_SDAT),
	.CLOCK_50					(clk),
	.reset						(~rst)
);

/* Instantiate the FFT IP to get the audio in the frequency domain - Added by Royce Hatley and Samuel Kanaan - comment out if just downloaded, FFT IP does not fully work yet */
/*
FFT_for_Equalizer fft_left_freq (
    .clk          (clk),            // clock input
    .reset_n      (rst),        // reset input
    .sink_valid   (1'd1),     // valid signal for sink
	 .sink_error   (1'd0),     // error signal
	 .sink_sop     (sink_sop), // start of sample
	 .sink_eop     (sink_eop), // end of sample
    .sink_real    (left_sample_q),      // real part of sink data
    .sink_imag    (16'b0),      // imaginary part of sink data
	 .inverse      (1'b1),		// 0 for forward FFT
	 .fftpts_in    (11'd1024), // fft size
    .source_ready (1'd1),   // ready signal for source
    .source_real  (left_FFT_sample_freq)    // real part of source data
);

FFT_for_Equalizer fft_right_freq (
    .clk          (clk),            // clock input
    .reset_n      (rst),        // reset input
    .sink_valid   (1'd1),     // valid signal for sink
	 .sink_error   (1'd0),     // error signal
	 .sink_sop     (sink_sop), // start of sample
	 .sink_eop     (sink_eop), // end of sample
    .sink_real    (right_sample_q),      // real part of sink data
    .sink_imag    (16'b0),      // imaginary part of sink data
	 .inverse      (1'b1),		// 0 for forward FFT
	 .fftpts_in    (11'd1024), // fft size
    .source_ready (1'd1),   // ready signal for source
    .source_real  (right_FFT_sample_freq)    // real part of source data
);
*/
/* Instantiate the FFT IP to get the audio back into time domain - Added by Royce Hatley and Samuel Kanaan */
/*
FFT_for_Equalizer fft_left_time (
    .clk          (clk),            // clock input
    .reset_n      (rst),        // reset input
    .sink_valid   (1'd1),     // valid signal for sink
	 .sink_error   (1'd0),     // error signal
	 .sink_sop     (sink_sop), // start of sample
	 .sink_eop     (sink_eop), // end of sample
    .sink_real    (left_FFT_sample_time),      // real part of sink data
    .sink_imag    (16'b0),      // imaginary part of sink data
	 .inverse      (1'b0),		// 1 for reverse FFT
	 .fftpts_in    (11'd1024), // fft size
    .source_ready (1'd1),   // ready signal for source
    .source_real  (left_sample_filter)    // real part of source data
);

FFT_for_Equalizer fft_right_time (
    .clk          (clk),            // clock input
    .reset_n      (rst),        // reset input
    .sink_valid   (1'd1),     // valid signal for sink
	 .sink_error   (1'd0),     // error signal
	 .sink_sop     (sink_sop), // start of sample
	 .sink_eop     (sink_eop), // end of sample
    .sink_real    (right_FFT_sample_time),      // real part of sink data
    .sink_imag    (16'b0),      // imaginary part of sink data
	 .inverse      (1'b0),		// 1 for reverse FFT
	 .fftpts_in    (11'd1024), // fft size
    .source_ready (1'd1),   // ready signal for source
    .source_real  (right_sample_filter)    // real part of source data
);

wire [15:0] left_FFT_sample_freq;
wire [15:0] right_FFT_sample_freq;

wire [15:0] left_FFT_sample_time;
wire [15:0] right_FFT_sample_time;

reg [1024:0] sample_counter;
wire sink_sop;
wire sink_eop;

always @(posedge clk or negedge rst) 
begin
    if (rst == 1'b0) 
	 begin
        sample_counter <= 1'd0;
    end 
	 else 
	 begin
        sample_counter <= (sample_counter == 1023) ? 0 : sample_counter + 1;
    end
end

assign sink_sop = (sample_counter == 0);      // first sample
assign sink_eop = (sample_counter == 1023);   // last sample
*/

/* Added wires for holding filtered audio - done by Royce Hatley and Samuel Kanaan  */

wire [15:0] left_sample_filter;
wire [15:0] right_sample_filter;

/* Instantiate the filter module - done by Royce Hatley and Samuel Kanaan */

filter filtering(clk, rst, equalize, SW, left_sample_q, right_sample_q, left_sample_filter, right_sample_filter);

/* Memory 32 bits wide for L and R with 32K (32768) spots */
/*reg [15:0]mem_address;
reg [31:0]idx;
reg mem_wren;
wire [15:0] left_sample_q;
wire [15:0] right_sample_q;
reg playing;
mem32K mem1(
	.address(mem_address),
	.clock(clk),
	.data({left_channel_audio_in, right_channel_audio_in}),
	.wren(mem_wren),
	.q({left_sample_q, right_sample_q})
	);*/

/* Memory 64 bits wide for L and R with 64K (65536) spots */
reg [15:0]mem_address;
reg [31:0]idx;
reg mem_wren;
wire [15:0] left_sample_q;
wire [15:0] right_sample_q;
reg playing;

mem64K mem1(
	.address(mem_address),
	.clock(clk),
	.data({left_channel_audio_in, right_channel_audio_in}),
	.wren(mem_wren),
	.q({left_sample_q, right_sample_q})
	);
	
reg [7:0]S;
reg [7:0]NS;
reg [31:0]last_sample_q;
reg [7:0]extra_writes;

assign LEDR[7:0] = S;
// Added LED to signal when the equalizer is being used - Added by Royce Hatley and Samuel Kanaan
assign LEDR[9] = equalize;

parameter
		//SAMPLE_SOUND_SIZE = 32'd32768;
		SAMPLE_SOUND_SIZE = 32'd65536; // 
		
parameter
		START = 			8'd0,
		WAIT = 			8'd1,
		WAIT_REC = 		8'd2,
		REC = 			8'd3,
		
		DELAY_1 	 = 	8'd5,
		DELAY_2	 = 	8'd6,
		DELAY_3	 = 	8'd7,
		WRITE_TO_MEM = 8'd8,
		WRITE_TO_MEM_DELAY_1 = 8'd9,
		INCREMENT_MEM_WRITE = 8'd10,
		
		TO_PLAY 					= 8'h0b,
		PLAY						= 8'h0c,
		LOAD_SAMPLE 			= 8'h0d,
		LOAD_SAMPLE_2 			= 8'h0e,
		LOAD_SAMPLE_3 			= 8'h0f,
		INCREMENT_MEM_READ 	= 8'h10,
		WAIT_COMPLETE_OUT_1 	= 8'h11,
		WAIT_COMPLETE_OUT_2	= 8'h12,
		ERROR 					= 8'hFF;

always @(posedge clk or negedge rst)
	if (rst == 1'b0)
	begin
		S <= START;
	end
	else
	begin
		S <= NS;
	end
	
always @(*)
	case (S)
		START: NS = WAIT;
		
		WAIT: 		if (KEY[1] == 1'b0)
							NS = WAIT_REC;
						else if (KEY[2] == 1'b0)
							NS = TO_PLAY;
						else
							NS = WAIT;
							
		WAIT_REC: 	if (KEY[1] == 1'b1) // wait until button released
							NS = REC;
						else
							NS = WAIT_REC;
							
		REC:			if (audio_in_available & audio_out_allowed)//audio_in_available == 1'b1)
							NS = DELAY_1;
						else
							NS = REC;
		DELAY_1: 	if (audio_in_available == 1'b0)
							NS = WRITE_TO_MEM;
						else
							NS = DELAY_1;
		WRITE_TO_MEM:NS = WRITE_TO_MEM_DELAY_1;
		WRITE_TO_MEM_DELAY_1: NS = INCREMENT_MEM_WRITE;
		INCREMENT_MEM_WRITE: 
						if (idx > SAMPLE_SOUND_SIZE)
							NS = WAIT;
						// NOTE the CLUDGE - 16.6666kHz output rate - you can see by just playing (SW[0] == 0) and seeing the writes and reads!!!
						else if (extra_writes < 2) // NOTE: PAJ Nov 13th - FIGURED OUT that the writing path in these modules is ~16.66kHz as in 1/3 the speed.  So the recording cludges this by writing the memory and copying the sample 3 times in the memory!!!
							NS = WRITE_TO_MEM;
						else 
							NS = REC;
		
		TO_PLAY: if (KEY[2] == 1'b1) // wait until button released
							NS = PLAY;
						else
							NS = TO_PLAY;
							
		PLAY:			if (idx >= SAMPLE_SOUND_SIZE)
							NS = WAIT;
						else if (audio_in_available & audio_out_allowed)//audio_out_allowed == 1'b1)
							NS = LOAD_SAMPLE;
						else
							NS = PLAY;			
		LOAD_SAMPLE: NS = LOAD_SAMPLE_2;
		LOAD_SAMPLE_2: NS = LOAD_SAMPLE_3;
		LOAD_SAMPLE_3: NS = INCREMENT_MEM_READ;
		INCREMENT_MEM_READ: NS = WAIT_COMPLETE_OUT_1;
		WAIT_COMPLETE_OUT_1: NS = WAIT_COMPLETE_OUT_2;
		WAIT_COMPLETE_OUT_2: NS = PLAY;
					
		default: NS = ERROR;
	endcase
				
				
always @(posedge clk or negedge rst)
	if (rst == 1'b0)
	begin
		mem_address <= 16'd0;
		mem_wren <= 1'b0;
		playing <= 1'b0;
		idx <= 32'd0;
		last_sample_q <= 32'd0;
		write_audio_out <= 1'b0;
		read_audio_in <= 1'b0;
		extra_writes <= 8'd0;
	end
	else
	begin
		case (S)
			WAIT:
			begin
				if (SW[0] == 1'b1)
				begin
					mem_address <= 16'd0;
					mem_wren <= 1'b0;
					playing <= 1'b0;
					idx <= 32'd0;
					read_audio_in <= 1'b0;
					write_audio_out <= 1'b0;
				end
				else
				begin
					mem_address <= 16'd0;
					mem_wren <= 1'b0;
					playing <= 1'b0;
					idx <= 32'd0;

					read_audio_in <= audio_in_available & audio_out_allowed;
					write_audio_out <= audio_in_available & audio_out_allowed;
				end				
			end

			REC:
			begin
				extra_writes <= 8'd0;
			end
			
			DELAY_1: read_audio_in <= 1'b1;
			//DELAY_2: read_audio_in <= 1'b0;
			
			WRITE_TO_MEM:
			begin
				mem_wren <= 1'b1;	
				read_audio_in <= 1'b0;
			end
			WRITE_TO_MEM_DELAY_1:
			begin
				mem_wren <= 1'b1;
				read_audio_in <= 1'b0;
			end
			INCREMENT_MEM_WRITE:
			begin
				mem_wren <= 1'b0;
				mem_address <= mem_address + 1'b1;
				idx <= idx + 1'b1;
				extra_writes <= extra_writes + 1'b1;
			end
				
			PLAY: playing <= 1'b1;
			INCREMENT_MEM_READ:
			begin
				write_audio_out <= 1'b1;
				last_sample_q <= {left_sample_filter, right_sample_filter}; //Add in filter samples to the last sample - modified by Royce Hatley and Samuel Kanaan
				mem_address <= mem_address + 1'b1;
				idx <= idx + 1'b1;
			end
			WAIT_COMPLETE_OUT_2: write_audio_out <= 1'b0;
						
		endcase
	end
	
endmodule

