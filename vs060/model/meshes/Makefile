all:
	for file in *.dae; do \
		echo $${file} ; \
		rosrun assimp_devel assimp export $$file `basename $${file} .dae`.stl; \
		ivcon `basename $${file} .dae`.stl `basename $${file} .dae`.stlb; \
		mv `basename $${file} .dae`.stlb `basename $${file} .dae`.stl; \
	done