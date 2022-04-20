/**
 * Copyright (c) 2022, Lehigh University
 * All rights reserved.
 * See COPYING for license.
 *
 * This file implements the batch processing of TSOAX commandline
 * program. The input images can be more than one, and the parameters
 * can vary.
 */

#include <sstream>
#include <iomanip>
#include <omp.h>
#include "boost/program_options.hpp"
#include "boost/filesystem.hpp"
#include "include/multisnake.h"
#include "include/snake_parameters.h"
#include "include/image_reader.h"
#include "include/image.h"

std::string ConstructSnakeFilename(const std::string &image_path,
                                   double ridge_threshold, double stretch);
std::string GetImageSuffix(const std::string &image_path);

int main(int argc, char **argv) {
  try {
    namespace po = boost::program_options;
    po::options_description generic("Generic options");
    generic.add_options()
        ("version,v", "Print version and exit")
        ("help,h", "Print help and exit");

    po::options_description required("Required options");
    required.add_options()
        ("image,i", po::value<std::string>()->required(),
         "Directory or path of input image files")
        ("parameter,p",
         po::value<std::string>()->required(),
         "Path of default parameter file")
        ("snake,s", po::value<std::string>()->required(),
         "Directory or path of output snake files");

    //soax::DataContainer ridge_range, stretch_range;
    po::options_description optional("Optional options");
    optional.add_options()
        //("invert", "Use inverted image intensity")
        //("nocut", "Output snakes before cutting at intersections and regrouping")
        //("nogroup", "Output snakes after cutting at intersections but without regrouping")
        ("numthreads,n", po::value<int>()->default_value(1),"Number of parallel threads to use in analysis");

    po::options_description all("Allowed options");
    all.add(generic).add(required).add(optional);
    po::variables_map vm;
    po::store(parse_command_line(argc, argv, all), vm);

    if (vm.count("version")) {
      const std::string version_msg(
          "Batch TSOAX 3.7.1\n"
          "Copyright (C) 2015-2022, Lehigh University.");
      std::cout << version_msg << std::endl;
      return EXIT_SUCCESS;
    }

    if (vm.count("help")) {
      std::cout << "TSOAX Commandline Mode: \n" << all;
      return EXIT_SUCCESS;
    }

    po::notify(vm);

    namespace fs = boost::filesystem;
    fs::path image_path(vm["image"].as<std::string>());
    if (!fs::exists(image_path)) {
      std::cerr << image_path << " does not exist. Abort." << std::endl;
      return EXIT_FAILURE;
    }

    fs::path parameter_path(vm["parameter"].as<std::string>());
    if (!fs::exists(parameter_path)) {
      std::cerr << parameter_path << " does not exist. Abort." << std::endl;
      return EXIT_FAILURE;
    }

    fs::path snake_path(vm["snake"].as<std::string>());
    std::string snake_path_name = snake_path.string();
    if (snake_path_name.back() != '/')
      snake_path_name += "/";
    if (!fs::exists(snake_path)) {
      std::cerr << snake_path << " does not exist. Abort." << std::endl;
      return EXIT_FAILURE;
    }
    
    int num_threads = vm["numthreads"].as<int>();
    
    std::cout << "Using " << num_threads << " threads" << std::endl;
    
    omp_set_num_threads(num_threads);

    try {
        std::cout << "Fixed parameters." << std::endl;
        
        if (fs::is_directory(image_path)) {      
          std::cout << "Input may contain multiple images." << std::endl;

          typedef std::vector<fs::path> Paths;
          Paths subfolder_paths;
          std::copy(fs::directory_iterator(image_path),
                    fs::directory_iterator(),
                    back_inserter(subfolder_paths));
                    
          //remove non-directories from subfolder_path so that parallel part does not have potentially very unbalanced loads
          for(int i = subfolder_paths.size()-1; i > -1; i--)   {
              fs::path subfolder = subfolder_paths[i];
              
              if (! fs::is_directory(subfolder)) {
                  subfolder_paths.erase(subfolder_paths.begin()+i);
              }
          }
          
          std::sort(subfolder_paths.begin(), subfolder_paths.end());
          
          
          // suppress output as in https://stackoverflow.com/questions/30184998/how-to-disable-cout-output-in-the-runtime
          std::cout.setstate(std::ios_base::failbit);
              
          // can this loop be made parallel? would need individual readers and multisnake to be defined in this loop
          #pragma omp parallel for schedule(dynamic, 1)
          for(int i = 0; i < subfolder_paths.size(); i++)   {
               fs::path subfolder = subfolder_paths[i];
        
               #pragma omp critical
                {
                    std::cout.clear();
                    std::cout << "Analyzing: " << subfolder.string() << std::endl;
                    std::cout.setstate(std::ios_base::failbit);
                }
                
               if (fs::is_directory(subfolder)) {
                    soax::Multisnake *multisnake = new soax::Multisnake;      
                    soax::ImageReader *reader = new soax::ImageReader;
                    
                    reader->ReadDir(QString::fromStdString(subfolder.string()));
                    
                    // load parameters into multisnake
                    multisnake->snake_parameters()->Load(QString::fromStdString(parameter_path.string()));                  

                    std::cout << "\nSegmentation started on " << subfolder.string()
                            << std::endl;
                    std::cout << "=========== Current Parameters ==========="
                            << std::endl;
      
                    std::cout << multisnake->snake_parameters()->ToString() << std::endl;


                    std::cout << "=========================================="
                            << std::endl;

                    time_t start, end;
                    time(&start);

                    multisnake->set_image(reader->GetImage(0));
                    multisnake->Initialize();

                    multisnake->DeleteConvergedSnakeSequence();



                    for (size_t i = 0; i < reader->GetNumberOfImages(); i++) {
                        multisnake->set_image(reader->GetImage(i));
                        multisnake->Initialize();

                        std::cout << "Extracting frame " << i << "..." << std::endl;
                        multisnake->Evolve();
                        if (multisnake->snake_parameters()->grouping()) {
                          multisnake->Reconfigure(i);
                        }
                        else {
                          multisnake->ReconfigureWithoutGrouping(i);
                        }
                    }

                    // solve for correspondence
                    multisnake->SolveCorrespondence(reader->GetNumberOfImages());

                    time(&end);
                    double time_elasped = difftime(end, start);

                    std::string path_str = subfolder.string();
                    std::string::size_type slash_pos = path_str.find_last_of("/\\");
                    std::string extracted_name = path_str.substr(
                      slash_pos+1);

                    std::ofstream outfile((snake_path_name + extracted_name + ".txt").c_str());

                    outfile << "image\t" << reader->GetFilePath().toStdString() << std::endl;
                    outfile << multisnake->snake_parameters()->ToString();
                    int dim = soax::GetImageDimension(reader->GetImage());
                    outfile << "dimension\t" << dim << std::endl;

                    for (size_t i = 0; i < reader->GetNumberOfImages(); i++) {
                    multisnake->set_image(reader->GetImage(i));
                    multisnake->SaveConvergedSnakes(i, outfile, dim);
                    }

                    multisnake->SaveConvergedSnakeTrack(outfile);
                    outfile.close();
                    multisnake->set_path(QString::fromStdString(snake_path_name + extracted_name + ".txt"));



                    std::cout << "Segmentation completed (Evolution time: "
                            << time_elasped << "s)" << std::endl;
                           
                    delete multisnake;
                    delete reader;
               }              
            }
            
            // re-enable output
            std::cout.clear();
            std::cout << "Finished batch job" << std::endl;
        }

        else {
          std::cout << image_path
                    << " exists, but is not a directory"
                    << std::endl;
        }
      
    } catch (const fs::filesystem_error &e) {
      std::cout << e.what() << std::endl;
      return EXIT_FAILURE;
    }
  } catch (std::exception &e) {
    std::cout << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}



std::string ConstructSnakeFilename(const std::string &image_path,
                                   double ridge_threshold, double stretch) {
  std::string::size_type slash_pos = image_path.find_last_of("/\\");
  std::string::size_type dot_pos = image_path.find_last_of(".");
  std::string extracted_name = image_path.substr(
      slash_pos+1, dot_pos-slash_pos-1);
  std::ostringstream buffer;
  buffer.precision(4);
  buffer << std::showpoint << extracted_name << "--ridge"
         << ridge_threshold << "--stretch" << stretch << ".txt";
  return buffer.str();
}


std::string GetImageSuffix(const std::string &image_path) {
  std::string::size_type dot_pos = image_path.find_last_of(".");
  return image_path.substr(dot_pos+1);
}
