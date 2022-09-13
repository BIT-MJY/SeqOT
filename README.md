# SeqOT
The code for our paper submitted to TIE: **SeqOT: A Spatial-Temporal Transformer Network for Place Recognition Using Sequential LiDAR Data.**

SeqOT is the sequence enhanced version of our previous work [OverlapTransformer (OT)](https://github.com/haomo-ai/OverlapTransformer) for RAL/IROS 2022.  

<img src="https://github.com/BIT-MJY/SeqOT/blob/main/overview.png" width="48%" ><img src="https://github.com/BIT-MJY/SeqOT/blob/main/visualize/SeqOT.gif" width="48%" >  
Fig. 1 System overview with Haomo dataset and visualized evaluation on NCLT dataset. It can be seen that SeqOT is robust to driving directions.  

### Table of Contents
1. [Dependencies](#Dependencies)
2. [How to use](#How-to-use)
3. [Data preparation](#Data-preparation)
4. [License](#License)

## Dependencies

We use pytorch-gpu for neural networks.

An nvidia GPU is needed for faster retrival.
OverlapTransformer is also fast enough when using the neural network on CPU.

To use a GPU, first you need to install the nvidia driver and CUDA.

- CUDA Installation guide: [link](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html)  
  We use CUDA 11.3 in our work. Other versions of CUDA are also supported but you should choose the corresponding torch version in the following Torch dependences.  

- System dependencies:

  ```bash
  sudo apt-get update 
  sudo apt-get install -y python3-pip python3-tk
  sudo -H pip3 install --upgrade pip
  ```
- Torch dependences:  
  Following this [link](https://pytorch.org/get-started/locally/), you can download Torch dependences by pip:
  ```bash
  pip3 install torch==1.10.2+cu113 torchvision==0.11.3+cu113 torchaudio==0.10.2+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html
  ```
  or by conda:
  ```bash
  conda install pytorch torchvision torchaudio cudatoolkit=11.3 -c pytorch
  ```  
  

- Other Python dependencies (may also work with different versions than mentioned in the requirements file):

  ```bash
  sudo -H pip3 install -r requirements.txt
  ```
  
## How to use
  
We provide a training and test tutorial for NCLT sequences in this repository. 
  
### Training

You can start the two-step training with

```
cd train
python training_seqot.py 
python gen_sub_descriptors.py
python training_gem.py 
```

### Test

The sub-descriptors of the database and query sequence have already been generated by `python gen_sub_descriptors.py` above. Here you can test SeqOT by

```
cd test
python test_gem_prepare.py
python test_seqot.py
```

Before training and test, please modify the params in the [config.yml](https://github.com/BIT-MJY/SeqOT/blob/main/config/config.yml)

##### data_root
* range_image_database_root: path of reference range images in database sequence.
* range_image_query_root: path of query range images in query sequence.
##### training_seqot
* traindata_file: index for training.
* poses_file: poses of the trajectory of the database.
* height: height of range images.
* width: width of range images.
* seqlen: sequence length to generate sub-descriptors.
* lr: learning rate for training feature extracter.
* resume: whether resuming from the pretrained model below.
* weights: pretrained feature extracter.
##### gen_sub_descriptors
* seqlen: sequence length to generate sub-descriptors.
* weights: pretrained feature extracter.
##### training_gem
* traindata_file: index for training (same as the one of training_seqot).
* poses_file: poses of the trajectory of the database (same as the one of training_seqot).
* descriptors_file: sub-descriptors of database for training (generated by gen_sub_descriptors.py).
* seqlen: sequence length to generate final-descriptors.
* lr: learning rate for training the pooling module.
* resume: whether resuming from the pretrained model below.
* weights: pretrained pooling module.
##### test_gem_prepare
* sub_descriptors_database_file: sub-descriptors of database scans (generated by gen_sub_descriptors.py).
* sub_descriptors_query_file: sub-descriptors of query scans (generated by gen_sub_descriptors.py).
* seqlen: sequence length to generate final-descriptors (same as the one of training_gem).
* weights: pretrained pooling module.
##### test_seqot
groud_truth_file: ground truth containing correct loops.
##### viz
* poses_database: poses of the trajectory of the database.
* poses_query: poses of the trajectory of the query sequence.

## Data preparation
Coming soon ...

## License

Copyright 2022, Junyi Ma, Xieyuanli Chen, Jingyi Xu, Guangming Xiong, Beijing Institute of Technology.

This project is free software made available under the MIT License. For more details see the LICENSE file.




