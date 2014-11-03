Abstract the prediction space e.g. classification-pmf, regression-pdf, etc.
Entropy calculation depends on Prediction space..

Classes: 
Descriptor
Labes - abstract to include several types of labes, including e.g: category, attributes, density
Example - descriptor + label
Examples - vector<Example> + methods that operate on groups of descriptors and/or labels

Sampler - prepares objects of type Examples

Forest - array of Decision Trees
DecisionTree - array of Nodes

Node - interface to alow several types
--reservoir - only contains data in leafs
--test-function 

Notes on reservoir: 
a global reservoir avoids duplication of data amongst trees, 
however if node optimisation includes a kernel/mapping function of the original descriptors, 
then this is not possible as the same data inputted to several trees may change at the leafs. 

ObjectiveFunction
Optimisation - selects best split of the data, Input-ObjectiveFunction + Node, Output {LNode,RNode}
How to sample from the parameter space?
First sample uniformly from the features [1-N]
Then, for one feature
1. Uniformly within range of sample values in the set of examples - poor
2. Uniformly sample within percentiles
3. Build histogram and sample from pmf
4. sample split threshold directly from the descriptor values in the sample

WeakLearner - eg. axis aligned hyperplane

Main loop: 
For time()
--add(current_data)
--optimize(max_number_of_nodes_to_split_per_iteration)

To plot/graph in 2D/3D there is a tool called Gnuplot, which has an c++ iostream interface.
http://www.stahlke.org/dan/gnuplot-iostream/