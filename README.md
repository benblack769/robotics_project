### Install:

```
pip install svg-path pygame visilibity matplotlib
```

Must have g++ installed.


### Run

To run on the many_guards environment:

```
# Generate the full weightmap
python get_visibility_graph.py environments/many_guards.json
# Compile c++ code
(cd fast_game_calc; bash build.sh)
# create directory to put data
mkdir -p wm_img_dir/environments
# Run path learning algorithm
./fast_game_calc/game_calc environments/many_guards.json environments/many_guards.fullgraph.json 0
# or if you want to run without prior for unbiased approach, try:
# ./fast_game_calc/game_calc environments/many_guards.json environments/many_guards.fullgraph.json 1
# generate plot for report
cp wm_img_dir/environments/many_guards.json_report.csv .
python plot_weights.py many_guards.json_report.csv
# generate videos of results at checkpoints
python process_all.py
```

### Create new environment

1. Use any SVG editor to create map in svg format.
2. Run `svg_to_json.py` to turn svg file into easily redable .svg.json file
3. Create environment .json file (see `environments/small_env.json` for an example)

Congratulations, now you can start running the algorithm! See Run instructions above.
