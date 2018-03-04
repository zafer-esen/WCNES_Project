/*Import java.io for the log creation*/
importPackage(java.io);

/*Create the motes log file*/
mlog = new FileWriter(sim.getTitle() + ".mlog");

/*Set speed to normal aka to 100%*/
sim.setSpeedLimit(1.0);

var motes = sim.getMotes(), i=0, finalmsg = new Array(), min, sec, mse;

/*Wait for all the motes to start*/
while(i < motes.length){
	YIELD();
	if(msg.startsWith("Starting")){
		i++;
	}
}
log.log("All motes up and running\n");
mlog.write("#All motes up and running\n#ID, DELAUNAY start time, INITIALIZATION start time, REDELCA start time, REDELCA end time, POWER, PEAK MEMORY ALLOCATION, NEIGHBORS...\n");

/*Write to motes serial interface their coordinates*/
for(i=0; i<motes.length; ++i){
	var x = motes[i].getInterfaces().getPosition().getXCoordinate();
	var y = motes[i].getInterfaces().getPosition().getYCoordinate();
	message = x + "#" + y;
	write(motes[i], message);
}

/*Create the log array, a string for each mote*/
for(var j=0; j<motes.length; ++j)
	finalmsg.push((j+1)+"");

/*While there are motes still running keep waiting for messages*/
i=0;
while(i < motes.length){
	try{
		/*Wait for a mote to print a message*/
		YIELD();

		/*If a message is not in a specific format that means something went wrong!*/
		if(!( msg.startsWith("DELAUNAY")||
		   msg.startsWith("INITIALIZE")	||
		   msg.startsWith("REDELCA")	||
		   msg.startsWith("POWER")		||
		   msg.startsWith("MEMORY")		||
		   msg.startsWith("NEIGHBOR")	||
		   msg.startsWith("TEST")		||
		   msg.startsWith(".") ||
		   msg.startsWith("EDGE")	||
		   msg.startsWith("LOCAL")	||
		   msg.startsWith("DONE")		)
		){
			throw "Something went terribly wrong - Execution failure";
		}

	} catch(e){
		log.log("Exception: "+e.toString()+"\n");
        mlog.write("#Exception: " + e.toString()+"\n");
        break;
	}

	/*If a mote is done with its execution count it*/
	if(msg.startsWith("DONE")){
		i++;
		continue;
	}

	/*Append the transmition power or the received neighbor to the motes log string*/
	else if(msg.startsWith("NEIGHBOR") || msg.startsWith("MEMORY") || msg.startsWith("ENERGY_CPU") || msg.startsWith("ENERGY_TX") || msg.startsWith("ENERGY_RX")){
		var str = msg.split(" ");
		finalmsg[id-1] += ","+str[1];
		continue;
	}
   if  (msg.startsWith("INITIALIZE") || msg.startsWith("REDELCA")){
	   var str = msg.split(" ");
		finalmsg[id-1] += ","+str[1];
	   } 
	/*The messages starting with DELAUNAY, INITIALIZE and REDELCA are just loged with their times
	 *Format the time at which the message was printed*/
	min = Math.floor(time/60000000);
	sec = Math.floor((time-min*60000000)/1000000);
	mse = Math.floor((time-min*60000000-sec*1000000)/1000);

	/*Append the time to the motes log string*/
	finalmsg[id-1] += ","+min+":"+(sec<10?"0":"")+sec+"."+(mse<10?"00":(mse<100?"0":""))+mse;

	if(msg.startsWith("POWER")){
		var str = msg.split(" ");
		finalmsg[id-1] += ","+str[1];
	}
}


for(var j=0; j<finalmsg.length; ++j){
	log.log(finalmsg[j]+"\n");
	mlog.write(finalmsg[j]+"\n");
}
log.log("Simulation success\n");
mlog.write("#Simulation success\n");
mlog.close();
log.testOK();