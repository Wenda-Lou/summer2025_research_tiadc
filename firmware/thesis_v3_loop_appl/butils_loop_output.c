/* Output only. Included by the board adapter and native UART tests. */
#ifdef BUTILS_LOOP_FRAGMENT

static void board_loop_output_status(void)
{
    const adc_cal_loop_output_t *o = &g_board_loop.output;
    xil_printf("LOOP-OUTPUT status=%s samples=%u reference_free=1 coefficients=frozen method=learned_pulses",
               adc_cal_loop_output_status_name(o->status), (unsigned)o->n_samples);
    board_loop_put("tone_hz", o->tone_hz, 1.0);
    xil_printf(" period_samples=%u events=%u", o->period_samples, o->events);
    board_loop_put("validation_a_mdb", o->validation_db_a, 1000.0);
    board_loop_put("validation_b_mdb", o->validation_db_b, 1000.0);
    xil_printf("\r\n");
}

static void board_loop_output_export(void)
{
    const adc_cal_loop_output_t *o = &g_board_loop.output;
    if (!o->n_samples || o->status == ADC_CAL_LOOP_OUTPUT_UNAVAILABLE) {
        xil_printf("LOOP-SIGNAL-END result=ERROR reason=no-output\r\n");
        return;
    }
    xil_printf("LOOP-SIGNAL-BEGIN version=1 samples=%u status=%s sample_rate_hz=1300000000 coefficients=frozen\r\n",
               (unsigned)o->n_samples, adc_cal_loop_output_status_name(o->status));
    for (size_t i = 0; i < o->n_samples; ++i) {
        xil_printf("LOOP-SIGNAL-DATA %u,%ld,%ld\r\n", (unsigned)i,
                   (long)lround(g_board_loop.output_a[i]*1000.0),
                   (long)lround(g_board_loop.output_b[i]*1000.0));
    }
    xil_printf("LOOP-SIGNAL-END result=OK samples=%u status=%s\r\n",
               (unsigned)o->n_samples, adc_cal_loop_output_status_name(o->status));
}

#endif
