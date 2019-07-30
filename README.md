# pcd_extract

1. In your `catkin_workspace/src`, clone this repo:		
	1. `git clone https://github.com/UditSinghParihar/pcd_extract.git`		
	2. In base directory of workspace, build it:		
		1. `catkin_make -j4 --pkg pcd_extract`		

2. Included files deal with subscribing to multiple files synchronously and using them to publish newer topics or saving to local directory.		