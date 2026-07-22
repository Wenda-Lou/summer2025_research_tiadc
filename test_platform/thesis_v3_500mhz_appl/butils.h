/* ============================================================================
 *  console_cmds.h — Unified UART-command helpers for DMA + JESD204 debug      
 * ============================================================================
 *  This header combines the previously separate JESD‑centric command set       
 *  (spi/phy/link) with DMA & memory debugging commands.  Internal helpers are  
 *  kept inside the .c file; external code sees only the high‑level handlers.   
 *                                                                              
 *  Command grammar                                                             
 *  ──────────────────────────────────────────────────────────────────────────   
 *  Keyword Option Arguments                       Description                  
 *  ──────────────────────────────────────────────────────────────────────────   
 *  spi     -r    <reg16>                         Read AD9695 register          
 *          -w    <reg16> <data8>                Write AD9695 register         
 *                                                                              
 *  phy     -r    <off32>                         Read JESD PHY register        
 *          -w    <off32> <data32>               Write JESD PHY register       
 *                                                                              
 *  link    -r    <off32>                         Read JESD Link register       
 *          -w    <off32> <data32>               Write JESD Link register      
 *                                                                              
 *  dma     -r                                    Dump last DMA buffer          
 *          -w                                    Start DMA capture             
 *          -d                                    Reset DMA core                
 *          -c                                    Resume DMA core               
 *                                                                              
 *  dbg     -r    <off32>                         Read DMA S2MM register        
 *          -w    <off32> <data32>               Write DMA S2MM register       
 *                                                                              
 *  mem     -r    <addr32>                        Read arbitrary address        
 *          -w    <addr32> <data32>              Write arbitrary address   
 * 
 *  adc     status                            Display ADC status
 *
 *          -c      <ch0|ch1|ch2|ch3>          Select ADC channel for configuration
 *
 *          -offset                           Enter DC offset calibration menu
 *              on                            Enable DC offset calibration
 *              off                           Disable DC offset calibration
 *              status                        Display current calibration status
 *              back                          Return to UART command prompt
 *
 *          -timing [frames]                  Align repeated ADC captures against
 *                                            the uploaded DAC TXT reference
 *                                            (default: 20 frames)
 *
 *          -gain                             Enter ADC gain setting menu
 *              IFC                           Input full-scale control mode
 *                  set <1.36~2.04>           Set input full-scale
 *                                            (Vpp differential)
 *                  status                    Display current input full-scale
 *                  sweep                     Sweep the supported IFC range
 *                  back                      Return to gain menu
 *                  quit                      Exit gain setting menu
 *
 *              back                          Return to UART command prompt
 *
 *          -cal [frames]                    Measurement-only calibration against
 *                                           the uploaded DAC TXT (default 10)
 *          -cal offset                      Run software ADC offset calibration
 *          -cal gain                        Run standalone software gain calibration
 *                                           using uploaded DAC TXT alignment
 *          -cal status                      Display separate software gain and
 *                                           offset states and latest metrics
 *          -cal reset                       Reset both software coefficients
 *                                           and both loop states
 *
 *          -ref                             Display uploaded reference status
 *                                           and buffer information
 *          -ref diagnose                    Capture one ADC DMA frame and test
 *                                           spectrum/sample-order hypotheses
 * --------------------------------------------------------------------------  
 *  © 2025 Your Project Name — MIT License                                      
 * ==========================================================================*/

#ifndef CONSOLE_CMDS_H
#define CONSOLE_CMDS_H

#include <stdint.h>
#include <stdbool.h>

double adc_get_configured_sample_rate_hz(void);
double adc_get_effective_sample_rate_hz(void);
double adc_get_sample_rate_correction_factor(void);
bool adc_effective_sample_rate_is_valid(void);
bool adc_set_effective_sample_rate_hz(double rate_hz);

#define ADC_TIMING_DEFAULT_FRAMES       20U
#define ADC_TIMING_MAX_FRAMES           1000U
#define ADC_TIMING_INTERFRAME_DELAY_US  100000U

extern volatile uint8_t adc_sweep_active;

/* Top‑level dispatcher */
void handle_cmd(char *line);

/* Individual command families (exposed for legacy callers) */
void handle_spi_cmd (char *line);
void handle_phy_cmd (char *line);
void handle_link_cmd(char *line);
void handle_dma_cmd (char *line);
void handle_dma_dbg_cmd(char *line);
void handle_mem_cmd (char *line);
void handle_adc_gain_cmd(void);
void handle_adc_offset_cmd(void);
void handle_adc_calibration_cmd(uint32_t frame_count);
void handle_adc_reference_status_cmd(void);

/* Reusable ADC acquisition helpers. */
int  adc_capture_frame(void);
void adc_timing_capture(uint32_t frame_count);


#endif /* CONSOLE_CMDS_H */
