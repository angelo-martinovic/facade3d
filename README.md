# 3D all the way! #

![rendering.jpg](https://bitbucket.org/repo/deay7R/images/402738020-rendering.jpg)

This README would normally document whatever steps are necessary to get your application up and running.

### What is this repository for? ###

* Quick summary
* Version
* [Learn Markdown](https://bitbucket.org/tutorials/markdowndemo)

### How do I get set up? ###

* Summary of set up
* Configuration
* Dependencies
* Database configuration
* How to run tests
* Deployment instructions

### Contribution guidelines ###

* Writing tests
* Code review
* Other guidelines

### Who do I talk to? ###

* Repo owner or admin
* Other community or team contact

### Creating 3D facade from WA

DRAFT I WILL REWRITE AND UPDATE....

Result of matlab is set of hypothesized windows, balconies, walls and other objects on the facade. All of them are represented in a form of boxes. To make facades looking realistic, one need to consider that windows are behind the wall (they make holes). This boolean operation of is not a easy problem. For that, we use 3ds max to create holes concatenate our results and render facade in a nice way. Now, we will show how to run this code

First, Matlab exports our result to ```export/..``` as several files for each facade,
```facadeID_.mesh_bin```,
```facadeID_.mesh_bin```,
```facadeID_.mesh_bin```
as well as copyies image into into the same directory. These are inputs for 3ds.

Second, run 3ds max by simple opening scene file ```.. .max``` and using MaxScript->OpenScript as in,

![2run_script.jpg](https://bitbucket.org/repo/deay7R/images/4177371873-2run_script.jpg)

op[en ```.ms```

![3gui.jpg](https://bitbucket.org/repo/deay7R/images/2092428085-3gui.jpg)


![43ds.jpg](https://bitbucket.org/repo/deay7R/images/912925717-43ds.jpg)


![5render.jpg](https://bitbucket.org/repo/deay7R/images/671782614-5render.jpg)