# RESEARCH_TRACK_2 Assignments

This README provides information about the assignments and the various components associated with it, including proper commenting, a Jupyter notebook for simulation interaction, and statistical analysis of the first RT1 assignment.

## Proper Commenting (Sphinx and/or Doxygen)

Proper commenting is crucial for maintaining code readability, understanding, and maintainability. In this assignment, I emphasize the use of Sphinx and/or Doxygen for generating documentation from the comments in the code.

To ensure proper commenting, follow these guidelines:

1. Use meaningful variable, function, and class names to enhance code comprehension.
2. Include comments to explain the purpose and functionality of complex code sections, algorithms, or design decisions.
3. Use inline comments for short explanations of code segments that might be less clear.
4. Include a comment header for each module, class, and function, describing their purpose, inputs, outputs, and any important considerations.
5. Follow the Sphinx or Doxygen syntax for documenting your code. Include information such as parameter descriptions, return values, and example usage.

By adhering to these commenting guidelines, I ensure that our code is self-explanatory and facilitates collaboration.

You can find the first RT1 assignment [documantation here.](https://giangalv.github.io/RESEARCH_TRACK_2/functions.html) 


## Jupyter Notebook for Simulation Interaction

To provide an interactive simulation experience for the 2nd assignment, I have created a Jupyter notebook. The notebook allows users to visualize and interact with the simulation environment.

To use the Jupyter notebook:

1. Install Jupyter notebook using the following command:

```bash
pip install jupyter
```

2. Launch the Jupyter notebook server:

```bash
jupyter notebook
```

3. Access the notebook through the web browser by following the link provided in the terminal.

4. Open the notebook file named `Client_Jup.ipynb`, inside the repository `/ros_simulation/scripts`.

5. Execute the cells in the notebook to interact with the simulation. The notebook provides instructions and explanations throughout the process.

The Jupyter notebook serves as a convenient tool to understand and experiment with the simulation of the 1st RT1 assignment.

If you want to Launch the simulation and see the homework, you have firstly launch the jupyter file, how I explaned before and after running the `roscore`, start the launch file:

```bash
roslaunch ros_simulation assingment1.launch
```

## Statistical Analysis of the First RT1 Assignment

The statistical analysis of the first RT1 assignment involves comparing the performance of two different implementations: my solution and a solution provided by one of your colleagues. The analysis aims to determine which implementation performs better when silver and golden tokens are randomly placed in the environment.

To perform the statistical analysis:

1. Ensure you have collected the required data from both implementations. The data should include relevant metrics such as execution time, number of successful token retrievals, or any other performance indicators.

2. Preprocess the collected data to ensure it is in a suitable format for analysis. This may involve cleaning, transforming, or aggregating the data.

3. Choose appropriate statistical tests to compare the two implementations. The choice of tests depends on the nature of the data and the research questions being addressed. Common tests include t-tests, ANOVA, or non-parametric tests.

4. Perform the statistical tests using a suitable statistical analysis tool, such as Python libraries like SciPy or statsmodels.

5. Analyze the results of the statistical tests and draw conclusions regarding the performance comparison of the two implementations. Consider factors such as statistical significance, effect size, and practical implications.

6. Document the entire statistical analysis process, including the data preprocessing steps, chosen statistical tests, and the obtained results, in a separate report or notebook.

Performing a thorough statistical analysis provides insights into the relative performance of different implementations and helps make informed decisions based on empirical evidence.
The reports' analysys is `s5521188_RT2.pdf` where I explain all the process.

## Research line
Human-drone interaction has emerged as a fascinating area of research and development in recent years. As drones become increasingly prevalent in various fields, such as aerial photography, surveillance, delivery services, and even entertainment, there is a growing need to establish efficient and intuitive methods of communication between humans and drones. This has led to the exploration of cutting-edge interfaces that revolutionize human-drone interaction. State-of-the-art research in this field focuses on developing innovative interfaces that enhance the control, communication, and collaboration between humans and drones. 

The revolutionizing human-drone interaction research line is pushing the boundaries of what is possible, creating cutting-edge interfaces that bridge the gap between humans and drones. These advancements are shaping the future of various industries, opening up new possibilities for safer, more intuitive, and efficient drone operation in diverse applications.

## Conclusion

This README provides an overview of the various components associated with the 1st and 2nd RT2 assignment. It emphasizes the importance of proper commenting using Sphinx and/or Doxygen, provides instructions for using the Jupyter notebook for simulation interaction, outlines the steps involved in performing a statistical analysis and what is an research line. By following these guidelines and utilizing the provided tools, you can enhance code quality, interact with the simulation, gain insights through statistical analysis and human-drone interaction.
