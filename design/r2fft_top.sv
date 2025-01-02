module r2fft_top
    #(
      parameter FFT_LENGTH = 1024, // FFT Frame Length, 2^N
      parameter FFT_DW = 16,       // Data Bitwidth
      parameter PL_DEPTH = 3,      // Pipeline Stage Depth Configuration (0 - 3)
      parameter FFT_N = $clog2( FFT_LENGTH ) // Don't override this
      )
     (
      // system
      input wire 			    clk,
      input wire 			    rst,
  
      // control
      input wire 			    autorun,
      input wire 			    run,
      input wire 			    fin,
      input wire 			    ifft,
      
      // status
      output wire 		            done,
      output wire [2:0] 		    status,
      output wire signed [7:0] 	    bfpexp,
  
      // input stream
      input wire 			          sact_istream,
      input wire signed [FFT_DW-1:0]  sdw_istream_real,
      input wire signed [FFT_DW-1:0]  sdw_istream_imag,
  
      // output / DMA bus
      input wire 			        dmaact,
      input wire [FFT_N-1:0] 	    dmaa,
      output wire signed [FFT_DW-1:0] dmadr_real,
      output wire signed [FFT_DW-1:0] dmadr_imag
     
      );

   // twiddle factor rom
   wire 		    twact;
   wire [FFT_N-1-2:0] 	    twa;
   wire [FFT_DW-1:0] 	    twdr_cos;
   
   // block ram0
   wire 		    ract_ram0;
   wire [FFT_N-1-1:0] 	    ra_ram0;
   wire [FFT_DW*2-1:0] 	    rdr_ram0;
   
   wire 		    wact_ram0;
   wire [FFT_N-1-1:0] 	    wa_ram0;
   wire [FFT_DW*2-1:0] 	    wdw_ram0;
   
   // block ram1
   wire 		    ract_ram1;
   wire [FFT_N-1-1:0] 	    ra_ram1;
   wire [FFT_DW*2-1:0] 	    rdr_ram1;
   
   wire 		    wact_ram1;
   wire [FFT_N-1-1:0] 	    wa_ram1;
   wire [FFT_DW*2-1:0] 	    wdw_ram1;
   
   R2FFT
     #(
       .FFT_LENGTH(FFT_LENGTH),
       .FFT_DW(FFT_DW),
       .PL_DEPTH(PL_DEPTH)
       )
   uR2FFT
     (
      .clk( clk ),
      .rst( rst ),

      .autorun( autorun ),
      .run( run ),
      .fin( fin ),
      .ifft( ifft ),

      .done( done ),
      .status( status ),
      .bfpexp( bfpexp ),

      .sact_istream( sact_istream ),
      .sdw_istream_real( sdw_istream_real ),
      .sdw_istream_imag( sdw_istream_imag ),

      .dmaact( dmaact ),
      .dmaa( dmaa ),
      .dmadr_real( dmadr_real ),
      .dmadr_imag( dmadr_imag ),

      .twact( twact ),
      .twa( twa ),
      .twdr_cos( twdr_cos ),

      .ract_ram0( ract_ram0 ),
      .ra_ram0( ra_ram0 ),
      .rdr_ram0( rdr_ram0 ),

      .wact_ram0( wact_ram0 ),
      .wa_ram0( wa_ram0 ),
      .wdw_ram0( wdw_ram0 ),

      .ract_ram1( ract_ram1 ),
      .ra_ram1( ra_ram1 ),
      .rdr_ram1( rdr_ram1 ),

      .wact_ram1( wact_ram1 ),
      .wa_ram1( wa_ram1 ),
      .wdw_ram1( wdw_ram1 )      
      
      );

   twrom
     #(
       .FFT_LENGTH(FFT_LENGTH),
       .FFT_DW(FFT_DW)
       )
     utwrom
       (
	.clk( clk ),
	.twact( twact ),
	.twa( twa ),
	.twdr_cos( twdr_cos )
	);


   dpram
     #(
       .ADDR_WIDTH(FFT_N-1),
       .DATA_WIDTH(FFT_DW*2)
       )
   ram0
     (
      .clk( clk ),
      
      .ract( ract_ram0 ),
      .ra( ra_ram0 ),
      .rdr( rdr_ram0 ),

      .wact( wact_ram0 ),
      .wa( wa_ram0 ),
      .wdw( wdw_ram0 )
      
      );
   

   dpram
     #(
       .ADDR_WIDTH(FFT_N-1),
       .DATA_WIDTH(FFT_DW*2)
       )
   ram1
     (
      .clk( clk ),
      
      .ract( ract_ram1 ),
      .ra( ra_ram1 ),
      .rdr( rdr_ram1 ),

      .wact( wact_ram1 ),
      .wa( wa_ram1 ),
      .wdw( wdw_ram1 )
      
      );

endmodule
