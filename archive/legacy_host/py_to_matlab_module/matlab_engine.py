# this is just to test if the matlab engine works or not
import os
import matlab
import matlab.engine as matlab_eng
import matplotlib.pyplot
import numpy as np

def main():
    # 1) start matlab
    print("Starting MATLAB")
    eng = matlab_eng.start_matlab()

    # Open MATLAB desktop
    # eng.desktop(nargout = 0)

    # Add current working directory to eng path
    work_dir = os.getcwd()
    eng.addpath(work_dir, nargout = 0)

    proj_root = r"D:\tiadc_cali_algorithm_matlab\matlab_eng_test"
    eng.addpath(proj_root, nargout = 0)
    eng.addpath(eng.genpath(proj_root), nargout = 0) # Recursively add to the engine path
    # Through this way we add this folder and all subfolders

    print(eng.which("test1", nargout = 1)) # if path successfully added, should return the full path

    # prepare variables
    PERIOD = 100
    TIME_CONSTANTS = 0.217 * PERIOD
    START_t = 900.0
    END_t = START_t + PERIOD * 4
    NUM_PLOTS = 3000

    dither_plot = eng.test1(TIME_CONSTANTS, PERIOD, START_t, END_t, NUM_PLOTS)

    dither_plot = np.array(dither_plot).ravel()

    np.savetxt("dither_plot.txt", dither_plot, fmt = "%.3f", delimiter="\t",
           header="t\ty", comments="")


    eng.eval("set(0,'DefaultFigureVisible','on');", nargout=0) # force to generate the plot
    eng.eval("drawnow; shg; uiwait(gcf);", nargout=0) # To wait until user closed the generated plot



if __name__ == "__main__":
    main()
