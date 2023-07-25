/*!
  Esto es un esqueleto de programa para usar en las prácticas
  de Visión Artificial.

  Se supone que se utilizará OpenCV.

  Para compilar, puedes ejecutar:
    g++ -Wall -o esqueleto esqueleto.cc `pkg-config opencv --cflags --libs`

*/

#include <iostream>
#include <exception>

//Includes para OpenCV, Descomentar según los módulo utilizados.
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>

#include "common_code.hpp"

const cv::String keys =
    "{help h usage ? |      | print this message.}"
    "{verbose        |      | activate verbose mode.}"
    "{i intrinsics   |      | Calibrate only extrinsics parameters. Using intrinsics from given file (-i=intr-file).}"
    "{s size         |<none>| square size.}"
    "{r rows         |<none>| number of board's rows.}"
    "{c cols         |<none>| number of board's cols.}"
    "{@output        |<none>| filename for output intrinsics file.}"
    "{@input1        |<none>| first board's view.}"
    "{@input2        |      | second board's view.}"
    "{@inputn        |      | ... n-idx board's view.}"
    ;

int
main (int argc, char* const* argv)
{
    int retCode=EXIT_SUCCESS;

    try {        
        cv::CommandLineParser parser(argc, argv, keys);
        parser.about("Calibrate the intrinsics parameters of a camera.");
        if (parser.has("help"))
        {
            parser.printMessage();
            return EXIT_SUCCESS;
        }
        float square_size = parser.get<float>("s");
        int rows = parser.get<int>("r");
        int cols = parser.get<int>("c");
        bool verbose = parser.has("verbose");
        std::string output_fname = parser.get<cv::String>("@output");
        if (!parser.check())
        {
            parser.printErrors();
            return EXIT_FAILURE;
        }

        //Get the inputs.
        //find the second argument without '-' at begin.
        int input = 1;
        bool found = false;
        while (input<argc && !found)
            found = argv[input++][0] != '-';
        CV_Assert(input<argc);
        std::vector<std::string> input_fnames;        
        for (; input<argc; ++input)
            input_fnames.push_back(std::string(argv[input]));

        //TODO




        if (parser.has("i"))
        {
            //TODO
            //Make extrinsic calibration.
            //Remenber: only one view is needed.





            //
            if (verbose)
            {
                //TODO
                //Show WCS axis.


                //
            }
        }
        else
        {
            //TODO
            //Make an intrisic calibration.
            //Remember: For each view (at least two) you must find the
            //chessboard to get the 3D -> 2D matches.


            //

            if (verbose)
            {
                //TODO
                //Show WCS axis on each pattern view.


                //
            }
        }
    }
    catch (std::exception& e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
