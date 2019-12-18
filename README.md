## **Sensor Fusion Algorithm**
---
### ORGANIZATION

#### Carleton University
---
### AUTHORS
Developed By: Rishabh Shrivastav, Shivam Goel, Kunal Markan, Prabhjot Singh<br/>
Instructed By: Cristina Ruiz Martin

---
### PROJECT DESCRIPTION

This project collects data from various sensors (some of the values may contain error). The collected data is passed through a series of computaional steps to obtain a fused value as per the algorithm. We also get a list of sensors whose values are out of range and sensors that are misbehaving. This project implementation makes use of external library GSL, which makes the handling of complex mathematical problems and equations easy to compute. The algorithm gathers raw input data from 'n' sensors. In our implementation we have kept the number of sensors to be 3 in number, however, it can be scaled to any number of sensors by just altering the sensor number value in the header file 'main.h' in /include folder. The software architecture is also defined in the documentation provided alongwith. The algorithm starts with parsing the input file which contains a list of sensors along with their respective data readings at specific times. The algorithm first computes a support degree matrix which is calculated using GSL library functions. After this, we calculate the corresponding eigenvalues and eigenvectors for the degree matrix coefficients. Then, the principal components and the contribution rate is calculated using functions defined in the algorithm. Finally, a degree score is calculated for each sensor which is used to eliminate the incorrect data, i.e, the misbehaving sensor value is omitted. The fused output is returned by the algorithm which is then stored in an output file in a format as prescribed in the problem statement. The input and output file as of the extension .csv type and the output is generated at run-time as expected. 

The algorithm is an implementation of the following paper:

G. Hongyan, "A simple multi-sensor data fusion algorithm based on principal component analysis," 2009 ISECS International Colloquium on Computing, Communication, Control, and Management, Sanya, 2009, pp. 423-426.
---

### FILE STRUCTURE

##### data [This folder contains the input and output files for the algorithm]
1. input.csv
2. output.csv

##### doc [This folder contains the documents for the project]
1. ducumentation.pdf
2. software_architecture.png

##### include[This folder contains the header files used in the program]
1. main.h

##### lib [This folder contains third party libraries required in the project]
1. gsl
	- 2.6
	  - ...
2. linker
	- libgsl.a
	- libgslcblas.a
	- pkgconfig
	  - gsl.pc

##### src [This folder contains the source file written in c for the project]
1. program_final.c

##### test [This folder the unit test for the different include files]
1. data [This folder contains the data files for unit tests]
2. src [This folder contains the source files for unit tests]

##### makefile [This file is used to run the program
]
##### README.md [This file has the readme information about the project]

### INSTRUCTIONS TO RUN/COMPILE THE PROJECT

1. documeantation.pdf inside the doc folder contains the explanation of this project.
	
2. **To run the project**
	1. Open the terminal.

	2. Set the command prompt in the Sensor_Fusion_Algorithm folder. To do so, type in the terminal the path to this folder.

		> Example: cd ../../Sensor_Fusion_Algorithm

	3. To compile the project, type in the terminal:
	
		> make clean; make all;
	
	4. To run the algorithm, go the folder with executable files (i.e., bin folder). To do so, type in the terminal the path to this folder.
	
		> Example: cd bin
	
	5. To check the output of the computation, open "../data/output.csv"

	6. To execute the results with different inputs
		- Create a new .csv file with the same structure as input.csv in /data folder
		- Run the algorithm by just chanding the path of the input file in the code to the new file name path
			> Example: FILE *fp = fopen("../data/new_input.csv", "r");
