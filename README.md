# centauro_cartesio
Package containg CartesI/O addons relared with the Centauro robot

## Build with hhcm-forest
This is a pure cmake package. You can build it easily with the `forest` tool.
```
pip install hhcm-forest
mkdir my_ws && cd my_ws
forest init
source setup.bash
forest add-recipes git@github.com:advrhumanoids/multidof_recipes.git --tag master
forest grow centauro_cartesio -j8
```
Optional: source the created workspace from your .bashrc

## Usage
An example launch file is available as `mon launch centauro_cartesio centauro_car_model.launch [gui:=true]`
