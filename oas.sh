gnome-terminal --window -e 'bash -c "source ~/bebop_ws/devel/setup.bash; roslaunch bebop_driver bebop_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; source devel/setup.bash; roslaunch bebop2 position.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; source devel/setup.bash; cd src/bebop2/scripts/; python sc-sfm.py ; exec bash"' \
