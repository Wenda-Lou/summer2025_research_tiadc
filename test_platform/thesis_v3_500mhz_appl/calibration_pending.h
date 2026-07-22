#ifndef CALIBRATION_PENDING_H
#define CALIBRATION_PENDING_H

/*
 * Invalidate the one-shot aligned frame retained by adc -cal.  State-changing
 * modules call this without depending on the pending frame's implementation.
 */
void calibration_pending_frame_invalidate(void);

#endif /* CALIBRATION_PENDING_H */
