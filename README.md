[Paul C. Merrell's Model Synthesis](http://graphics.stanford.edu/~pmerrell/thesis.pdf)
implemented in minetest. This mod is unfinished.

# How to use

## Collect model information

* Build an input model in the sky
* Select a WorldEdit region (set the two positions) around the built model;
  there should be many air nodes included at the borders of the region.
* Run the `//gen_mi <name>` chatcommand


## Generate/Synthesize a model

* Select some WorldEdit region; it should be big enough for the model to fit in
* Run the `//synth <name>` chatcommand


# Example Screenshots

Input:
![fot_2020-02-24-20:38:53_374444728](https://user-images.githubusercontent.com/3192173/75186316-fa308600-5747-11ea-8875-3f1cb41aa9d2.jpg)

Generated output:
![modelsynth_2x3x2_broads](https://user-images.githubusercontent.com/3192173/75351928-33ccd280-58a9-11ea-86df-89aab6d981d1.jpg)
I'm not sure if the road ends should appear in the output even though there are
no road ends in the input.
