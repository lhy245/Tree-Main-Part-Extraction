<img width="327" height="369" alt="example" src="https://github.com/user-attachments/assets/674aeb38-9760-4a0e-be59-9a7fd233febb" />This project contains the code and data for the paper "Houyu Liang, Xiang Zhou et al., Tree Main-Part Extraction: A Method for Individual Tree Point Cloud and Parameter Extraction in Complex Forest Environments Integrating Structural Characteristics and Growth Constraint Rules". The main function of this code is to perform individual tree extraction from MLS forest point clouds.

The code has been tested on Windows systems and can run successfully in Python 3.8.

Please read the 'requirements.txt' file to install the necessary libraries.

We also provide a simple sample dataset. This sample dataset is from: "Wielgosz, M., Puliti, S., Xiang, B., Schindler, K., Astrup, R., 2024. SegmentAnyTree: A sensor and platform agnostic deep learning model for tree segmentation using laser scanning data. Remote Sens. Environ. 313, 114367. ". Links to other result data can be found at ×. The provided algorithm code is divided into three parts:


1.	Branch and Leaf Point Filtering Part:
•	Function: Filters branches, leaves, understory vegetation, and ground points from the forest point cloud to extract the main stem (tree trunk).
•	Input:
o	Input file path: Point cloud in LAS format. Forest plot (normalization is recommended beforehand). E.g., G:/computer/study/singtree/code_web/data/example.las
•	 Output: 
o	output file path: Point cloud in LAS format. This is the trunk point cloud after filtering out branches, leaves, understory vegetation, and ground points. E.g., G:/computer/study/singtree/code_web/data/gan.las


2.	 Individual Tree Segmentation Part: 
•	 Function:  Segments the filtered trunk point cloud to generate individual tree trunk point clouds.
•	 Input: 
o	Input file path: Point cloud in LAS format. This is the main stem point cloud output from Step 1. E.g., G:/computer/study/singtree/code_web/data/gan.las
•	 Output: 
o	output file path: Point cloud in LAS format. This is the individual tree point cloud after segmenting the main stem part. E.g., G:/computer/study/singtree/code_web/data/single.las

3.	 Trunk Repair and Top Point Cloud Reconstruction: 
•	 Function:  Repairs the individual tree trunk point cloud and reconstructs the top crown point cloud.
•	 Input: 
o	Input file path1: Point cloud in LAS format. This is the individual tree trunk point cloud output from Step 2. E.g., G:/computer/study/singtree/code_web/data/single.las
o	Input file path2: Point cloud in LAS format. This is the original point cloud input in Step 1. E.g., G:/computer/study/singtree/code_web/data/example.las
•	 Output: 
o	output file path: Point cloud in LAS format. This contains the complete trunk and the top crown points of the individual tree. E.g., G:/computer/study/singtree/code_web/data/individual_tree.las

