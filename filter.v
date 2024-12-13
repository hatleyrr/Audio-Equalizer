/* Made by Royce Hatley and Samuel Kanaan

	This module will be instantiated to filter the sample audio for equalization 
	
	Algorithm inspired by Verilog filter implementation example at this link: https://www.unrepo.com/verilog/verilog-for-digital-signal-processing */

module filter(clk, rst, start, SW, left_audio_in, right_audio_in, left_audio_out, right_audio_out);
input clk, rst;
input start;
input [9:0] SW;
input [15:0] left_audio_in;
input [15:0] right_audio_in;
output reg [15:0] left_audio_out;
output reg [15:0] right_audio_out;

reg [3:0] S;
reg [3:0] NS;

// State declaration
parameter START = 4'b0, 
			FILTER = 4'd1,
			ADJUST = 4'd2,
			DONE = 4'd3,
			DELAY = 4'd4,
			ERROR = 4'd5;
			
// Filter coefficient array - applied to the samples to filter them
reg [15:0] coe [2:0];

//Shift registers to hold audio samples
reg [15:0] left_shift[2:0];
reg [15:0] right_shift[2:0];

// Pitch adjuster regs
reg [15:0] pitch_divide;
reg [15:0] pitch_count;

// sum variables so that the audio can be added and scaled to 16 bits
reg [31:0] left_sum;
reg [31:0] right_sum;

//Variables for holding filtered samples
reg [15:0] left_hi;
reg [15:0] left_mid;
reg [15:0] left_lo;

reg [15:0] right_hi;
reg [15:0] right_mid;
reg [15:0] right_lo;

always@(posedge clk or negedge rst)
begin
	if (rst == 1'b0)
	begin
		S = START;
	end
	else
	begin
		S = NS;
	end
end

always@(*)
begin
	case(S)
		START: NS = (start == 1'd1) ? FILTER : START;
		FILTER: NS = ADJUST;
		ADJUST: NS = DONE;
		DONE: NS = (start == 1'd1) ? DONE : START;
		default: NS = START;
	endcase
end

//Filter logic
always@(posedge clk or negedge rst)
begin
	if (rst == 1'b0)
	begin
		left_shift[0] <= 16'd0; //Clear the register of samples
		left_shift[1] <= 16'd0;
		left_shift[2] <= 16'd0;
		
		right_shift[0] <= 16'd0;
		right_shift[1] <= 16'd0;
		right_shift[2] <= 16'd0;

		left_audio_out <= left_audio_in;    //Clear audio output
		right_audio_out <= right_audio_in;
		coe[0] <= 16'd2;
		coe[1] <= 16'd2;
		coe[2] <= 16'd2;
	end
	else
	begin
		case(S)
		START: begin
					left_shift[0] <= 16'd0; //Clear the register of samples
					left_shift[1] <= 16'd0;
					left_shift[2] <= 16'd0;
		
					right_shift[0] <= 16'd0;
					right_shift[1] <= 16'd0;
					right_shift[2] <= 16'd0;
		
					left_audio_out <= left_audio_in;    //Clear audio output
					right_audio_out <= right_audio_in;
				//Coefficients to apply to samples
				coe[0] <= 16'd2;
				coe[1] <= 16'd2;
				coe[2] <= 16'd2;
				end
		FILTER: begin
				left_shift[0] <= left_audio_in;  //Storing audio samples
				left_shift[1] <= left_shift[0];  //Shifting samples
				left_shift[2] <= left_shift[1];
				
				right_shift[0] <= right_audio_in; 
				right_shift[1] <= right_shift[0];
				right_shift[2] <= right_shift[1];
				end
		ADJUST: begin
					// set pitches for pitch divider - when the counter reaches the pitch_divide, thats the pitch of the band
					case (SW)
                  10'b0000000010: pitch_divide <= 16'd250;  // high-treble 
                  10'b0000000100: pitch_divide <= 16'd500;  // treble
                  10'b0000001000: pitch_divide <= 16'd750;  // low-treble
                  10'b0000010000: pitch_divide <= 16'd1000; // high-mid
                  10'b0000100000: pitch_divide <= 16'd1250; // mid
                  10'b0001000000: pitch_divide <= 16'd1500; // low-mid
                  10'b0010000000: pitch_divide <= 16'd1750; // high-bass
                  10'b0100000000: pitch_divide <= 16'd2000; // bass
						10'b1000000000: pitch_divide <= 16'd2500; // low-bass
					endcase
					
					left_hi <= coe[0] * left_shift[0];
               right_hi <= coe[0] * right_shift[0];
               left_mid <= coe[1] * left_shift[1];
               right_mid <= coe[1] * right_shift[1];
               left_lo <= coe[2] * left_shift[2];
               right_lo <= coe[2] * right_shift[2];
					
					left_sum <= left_hi + left_mid + left_lo;
					right_sum <= right_hi + right_mid + right_lo;
					end
		DONE: begin
				if (pitch_count == pitch_divide) 
				begin
					// the audio must be 32 bits - use if-statement to restrict left and right audio to 16 bits each
					left_audio_out <= (left_sum > 16'sd32767) ? 16'sd32767 : (left_sum < -16'sd32768) ? -16'sd32768 : left_sum[15:0];
					right_audio_out <= (right_sum > 16'sd32767) ? 16'sd32767 : (right_sum < -16'sd32768) ? -16'sd32768 : right_sum[15:0];
               pitch_count <= 16'd0;
				end 
				else 
				begin
               pitch_count <= pitch_count + 1;
					left_audio_out <= left_audio_in;
					right_audio_out <= right_audio_in;
            end
				
				end
		default: begin
					left_audio_out <= left_audio_in;
					right_audio_out <= right_audio_in;
					end
		endcase
	end
end
endmodule

