
### Ideas for Models & Methods

For inspiration, we prepared a few models and methods that you can use as a starting point. 

Ideas for Models:
- [2D-XY Bicycle](documentation/Model%20-%202D-XY%20Bicycle%20Control.md)
- [2D-XZ Drone](documentation/Model%20-%202D-XZ%20Drone%20Control.md)
- [2D-XZ Rocket](documentation/Model%20-%202D-XZ%20Rocket%20Control.md)

Ideas for Methods:
- Open Loop
	- [Open Loop Planning](documentation/Method%20-%20Open%20Loop%20Planning.md)
- Closed Loop
	- [Model Predictive Control](documentation/Method%20-%20Model%20Predictive%20Control.md)
	
For every of the models and the methods, we prepared examples, that you can use as a starting point.

### Tools & Installation

We use the python framework [`CasAdi`](https://web.casadi.org/) to formulate OCPs.
If you already have Python installed on your system or want to use another IDE, feel free to skip to bullet 4.

```bash
pip install numpy scipy matplotlib casadi
```

### Getting Started

First, please follow the installation instructions above.

1. If you have git installed, clone this repository using `git clone` into a folder of choice. Alternatively, you can just download the folder directly from Github.
2. Navigate to the cloned folder in your terminal
3. Run the example file
	```bash
	python examples/omniBotTrackingOCP.py
	```
	If you see a figure popping up, that means that everything works as intended.

