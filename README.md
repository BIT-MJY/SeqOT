# SeqOT
The code for our paper accepted by IEEE Transactions on Industrial Electronics:   
### SeqOT: A Spatial-Temporal Transformer Network for Place Recognition Using Sequential LiDAR Data.

SeqOT is a sequence-enhanced LiDAR-based place recognition method based on our previous work [OverlapTransformer (OT)](https://github.com/haomo-ai/OverlapTransformer) for RAL/IROS 2022.  

Developed by [Junyi Ma](https://github.com/BIT-MJY) and [Xieyuanli Chen](https://github.com/Chen-Xieyuanli).

---

<img src="https://github.com/BIT-MJY/SeqOT/blob/main/overview.png" width="48%" ><img src="https://github.com/BIT-MJY/SeqOT/blob/main/visualize/SeqOT.gif" width="48%" >  
Fig. 1 System overview with Haomo dataset and visualized evaluation on NCLT dataset. It can be seen that SeqOT is robust to driving directions.  

---
## News
**[2022-12]** Our paper is accepted by **IEEE Transactions on Industrial Electronics (TIE)**!



### Table of Contents
1. [Publications](#Publications)
2. [Dependencies](#Dependencies)
3. [How to use](#How-to-use)
4. [Data preparation](#Data-preparation)
5. [License](#License)

## Publications

If you use the code in your work, please cite our [paper](https://ieeexplore.ieee.org/document/9994714):

```
@ARTICLE{ma2022tie,
  author={Ma, Junyi and Chen, Xieyuanli and Xu, Jingyi and Xiong, Guangming},
  journal={IEEE Transactions on Industrial Electronics}, 
  title={SeqOT: A Spatial-Temporal Transformer Network for Place Recognition Using Sequential LiDAR Data}, 
  year={2022},
  doi={10.1109/TIE.2022.3229385}}
```


## Dependencies

We use pytorch-gpu for neural networks.

An nvidia GPU is needed for faster retrival.

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
Note that we only train our model using the oldest sequence of NCLT dataset (2012-10-08), to prove that our model works well for long time spans even if seeing limited data.  

We will release our pretrained model after the paper is accepted.

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
* traindata_file: index for training. ([link](https://drive.google.com/file/d/1jEcnuHjEi0wqe8GAoh6UTa4UXTu0sDPr/view?usp=share_link))
* poses_file: poses of the trajectory of the database. ([link](https://drive.google.com/file/d/1NusFV_xMI1s7i2s3u9vKdHnw0dWI7uii/view?usp=share_link))
* height: height of range images.
* width: width of range images.
* seqlen: sequence length to generate sub-descriptors.
* lr: learning rate for training feature extracter.
* resume: whether resuming from the pretrained model below.
* weights: pretrained feature extracter. ([link](https://drive.google.com/file/d/17EDuLzolOFjE7bq9BbwqDDnnWwiTlqWB/view?usp=share_link))
##### gen_sub_descriptors
* seqlen: sequence length to generate sub-descriptors.
* weights: pretrained feature extracter. ([link](https://drive.google.com/file/d/17EDuLzolOFjE7bq9BbwqDDnnWwiTlqWB/view?usp=share_link))
##### training_gem
* traindata_file: index for training (same as the one of training_seqot).
* poses_file: poses of the trajectory of the database (same as the one of training_seqot).
* descriptors_file: sub-descriptors of database for training (generated by gen_sub_descriptors.py).
* seqlen: sequence length to generate final-descriptors.
* lr: learning rate for training the pooling module.
* resume: whether resuming from the pretrained model below.
* weights: pretrained pooling module. ([link](https://drive.google.com/file/d/1MdwavdbegA6P739mKlC2xJAcmZSbVD6N/view?usp=share_link))
##### test_gem_prepare
* sub_descriptors_database_file: sub-descriptors of database scans (generated by gen_sub_descriptors.py).
* sub_descriptors_query_file: sub-descriptors of query scans (generated by gen_sub_descriptors.py).
* seqlen: sequence length to generate final-descriptors (same as the one of training_gem).
* weights: pretrained pooling module. ([link](https://drive.google.com/file/d/1MdwavdbegA6P739mKlC2xJAcmZSbVD6N/view?usp=share_link))
##### test_seqot
groud_truth_file: ground truth containing correct loops. ([link](https://drive.google.com/file/d/13-tpLQiHK4krd-womDV6UPevvHUIFNyF/view?usp=share_link))
##### viz
* poses_database: poses of the trajectory of the database. ([link](https://drive.google.com/file/d/1NusFV_xMI1s7i2s3u9vKdHnw0dWI7uii/view?usp=share_link))
* poses_query: poses of the trajectory of the query sequence. ([link](https://drive.google.com/file/d/1NBZeYK5giNr5r5l0TI0Ov9W_M8mvZT4a/view?usp=share_link))
##### gen_training_index
* poses_database: poses of the trajectory of the database. ([link](https://drive.google.com/file/d/1NusFV_xMI1s7i2s3u9vKdHnw0dWI7uii/view?usp=share_link))
* poses_query: poses of the trajectory of the query sequence. ([link](https://drive.google.com/file/d/1NBZeYK5giNr5r5l0TI0Ov9W_M8mvZT4a/view?usp=share_link))
* scan_database_root: path of reference .bin of the database. ([link](https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu/velodyne_data/2012-01-08_vel.tar.gz))


## Data preparation

Please use the following commands to prepare data. We have uploaded all the necessary files mentioned above except the range images which you can generate easily following our instruction.

### Range image generation

Please use [this script](https://github.com/BIT-MJY/SeqOT/blob/main/tools/utils/gen_depth_data.py) to generate range images of NCLT dataset. You need to modify the params including `scan_folder` and `dst_folder`.

### Training index generation

Please use [this script](https://github.com/BIT-MJY/SeqOT/blob/main/data_prepararion/gen_training_index.py) to generate training indices of NCLT dataset. You need to modify the params including `poses_database`, `poses_query`, and `scan_database_root`.

```
cd data_prepararion
python ./gen_training_index.py 
```

### Groud truth generation

Please use [this script](https://github.com/BIT-MJY/SeqOT/blob/main/data_prepararion/gen_ground_truth.py) to generate groud truth loops of NCLT dataset. 

```
cd data_prepararion
python ./gen_ground_truth.py 
```

## License

Copyright 2022, Junyi Ma, Xieyuanli Chen, Jingyi Xu, Guangming Xiong, Beijing Institute of Technology.

This project is free software made available under the MIT License. For more details see the LICENSE file.




