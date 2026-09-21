# Next steps discussed with prof, Sept 21st 2026

## 1. Keep testing to full scale output of DAC
-   Prof wants to know the low cutoff and high cutoff of the DAC output. Try different frequencies 

## 2. Manually introduce some mismatch to the 2 channels in the ADC and see how the calibration loop perform
-   record ground truth
-   know how each aspects are calculated (gain, offset, skew)
-   be familier of how the result comes out and what they mean

## 3. move the current calibration loop from host side to the ARM side
-   this should be done in parallel with #2

## 4. Set up the actual process pipeline
-   the dither comes from DAC, the tone comes from a generator
-   they are combined using a RF combiner
-   and go through a RF splitter to the 2 channels A/B

## 5. With everything above done, see if we can make the calibration loop converge faster