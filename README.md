# Ping Pong Catcher

_Demonstrations of catching a ping pong ball with a paddle welded to the end effector of an [iiwa](https://www.kuka.com/en-us/products/robotics-systems/industrial-robots/lbr-iiwa) with [Drake](https://drake.mit.edu/)._

See [report.pdf](./report.pdf) to see how it works.

## Dependencies

* Docker + `docker-compose`

## Usage

1. Run a container with Drake, PyDrake, and Jupyter Lab ready to go
```sh
UID=${UID} GID=${GID} docker-compose up --build
```

2. Visit the URL that appears in your browser. You should be able to open and run the notebooks in this repo. `demo.ipynb` is a good place to play with the code. The rest of the notebooks execute the experiments from the evaluation section of our report.
