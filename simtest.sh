for f in deployments/; do
	printf "\nnow parsing directory $f\n";
	for s in $f*.dat; do
		filename="${s##*/}";
		filename=(${filename//./ });
		printf "\t$filename: "
		./simgen_rtx ${filename[0]} $s z1
		ex=$?
		if [[ $ex == 0 ]];
			then
				printf "simulation file created successfully - "
			else
				printf "failed to create simulation file\n";
				exit 0
		fi
		re="[0-9]+"
		num=-1
		if [[ $filename =~ $re ]]; then num=${BASH_REMATCH[0]}; fi
			printf "now simulating"
			java -mx1024m -jar ~/development/contiki-2.7-seed2/tools/cooja/dist/cooja.jar -nogui="./$filename.csc" -contiki='../../../contiki-2.7-seed2' > $filename.slog
			mv $filename.csc *.mlog $f
			printf "\r\t****$filename simulation file created successfully - simulation successful - logs moved to directory\n";
	done
done
