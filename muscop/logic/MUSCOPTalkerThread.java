package muscop.logic;

import static muscop.pojo.State.*;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import com.esotericsoftware.kryo.io.Output;

import api.API;
import es.upv.grc.mapper.Location2DUTM;
import es.upv.grc.mapper.Location3DUTM;
import main.api.ArduSim;
import main.api.Copter;
import main.api.GUI;
import main.api.communications.CommLink;
import muscop.pojo.Message;

/** 
 * Thread used to send messages to other UAVs.
 * <p>Developed by: Francisco Jos&eacute; Fabra Collado, from GRC research group in Universitat Polit&egrave;cnica de Val&egrave;ncia (Valencia, Spain).</p> */

public class MUSCOPTalkerThread extends Thread {
	
	private AtomicInteger currentState;

	private long selfId;
	private boolean isMaster;
	private ArduSim ardusim;
	private Copter copter;
	private GUI gui;
	private byte[] outBuffer;
	private Output output;
	private byte[] message;
	private CommLink link;
	// Concurrency variables:
	private AtomicBoolean missionReceived;
	private AtomicInteger wpReachedSemaphore, moveSemaphore;
	
	private long cicleTime;		// Cicle time used for sending messages
	private long waitingTime;	// (ms) Time to wait between two sent messages
	
	private AtomicInteger indexMasterLocal;
	private long[] listaMasters;
	private AtomicBoolean iAmAlive;
	
	@SuppressWarnings("unused")
	private MUSCOPTalkerThread() {}

	public MUSCOPTalkerThread(int numUAV, boolean isMaster, long[] listaMasters,
			AtomicBoolean missionReceived, AtomicInteger wpReachedSemaphore, AtomicInteger moveSemaphore) {
		this.currentState = MUSCOPParam.state[numUAV];
		this.selfId = API.getCopter(numUAV).getID();
		this.isMaster = isMaster;
		this.ardusim = API.getArduSim();
		this.copter = API.getCopter(numUAV);
		this.gui = API.getGUI(numUAV);
		this.outBuffer = new byte[CommLink.DATAGRAM_MAX_LENGTH];
		this.output = new Output(outBuffer);
		this.link = API.getCommLink(numUAV);
		this.missionReceived = missionReceived;
		this.wpReachedSemaphore = wpReachedSemaphore;
		this.moveSemaphore = moveSemaphore;
		this.indexMasterLocal = MUSCOPParam.indexMaster[numUAV];
		this.listaMasters = listaMasters;
		this.iAmAlive = MUSCOPParam.alive[numUAV];
		
		this.cicleTime = 0;
	}

	@Override
	public void run() {
		
		/** SHARE MISSION PHASE */
		if (this.isMaster) {
			gui.logVerboseUAV(MUSCOPText.MASTER_DATA_TALKER);
			Location3DUTM[] mission;
			while ((mission = MUSCOPParam.missionSent.get()) == null) {
				ardusim.sleep(MUSCOPParam.STATE_CHANGE_TIMEOUT);
			}
			output.clear();
			output.writeShort(Message.DATA);
			output.writeShort(mission.length);
			Location3DUTM waypoint;
			for (int i = 0; i < mission.length; i++) {
				waypoint = mission[i];
				output.writeDouble(waypoint.x);
				output.writeDouble(waypoint.y);
				output.writeDouble(waypoint.z);
			}
			output.flush();
			message = Arrays.copyOf(outBuffer, output.position());
			
			cicleTime = System.currentTimeMillis();
			while (currentState.get() == SHARE_MISSION) {
				link.sendBroadcastMessage(message);
				
				// Timer
				cicleTime = cicleTime + MUSCOPParam.SENDING_TIMEOUT;
				waitingTime = cicleTime - System.currentTimeMillis();
				if (waitingTime > 0) {
					ardusim.sleep(waitingTime);
				}
			}
		} else {
			gui.logVerboseUAV(MUSCOPText.SLAVE_WAIT_LIST_TALKER);
			output.clear();
			output.writeShort(Message.DATA_ACK);
			output.writeLong(selfId);
			output.flush();
			message = Arrays.copyOf(outBuffer, output.position());
			
			cicleTime = System.currentTimeMillis();
			while (currentState.get() == SHARE_MISSION) {
				if (missionReceived.get()) {
					link.sendBroadcastMessage(message);
				}
				
				// Timer
				cicleTime = cicleTime + MUSCOPParam.SENDING_TIMEOUT;
				waitingTime = cicleTime - System.currentTimeMillis();
				if (waitingTime > 0) {
					ardusim.sleep(waitingTime);
				}
			}
		}
		while (currentState.get() < TAKING_OFF) {
			ardusim.sleep(MUSCOPParam.STATE_CHANGE_TIMEOUT);
		}
		
		/** TAKING OFF and SETUP FINISHED PHASES */
		gui.logVerboseUAV(MUSCOPText.TALKER_WAITING);
		while (currentState.get() < FOLLOWING_MISSION) {
			ardusim.sleep(MUSCOPParam.STATE_CHANGE_TIMEOUT);
		}
		
		/** COMBINED PHASE MOVE_TO_WP & WP_REACHED */
		int currentWP = 0;
		while (currentState.get() == FOLLOWING_MISSION) {
			/** WP_REACHED PHASE */
			if (listaMasters[indexMasterLocal.get()] == selfId) {
				
				if(selfId==2 && currentWP == 1) {
					gui.logUAV("El 2 Maestro Muere");
					return;//Para matar al primer maestro
				}
				if(selfId==3 && currentWP == 2) {
					gui.logUAV("El 3 Maestro Muere");
					return;//Para matar al primer maestro
				}
				if(selfId==1 && currentWP == 3) {
					gui.logUAV("El 1 Maestro Muere");
					return;//Para matar al primer maestro
				}
				if(selfId==4 && currentWP == 4) {
					gui.logUAV("El 1 Maestro Muere");
					return;//Para matar al primer maestro
				}
				if(!iAmAlive.get()) {
					return;
				}
				gui.logVerboseUAV(MUSCOPText.TALKER_WAITING);
				output.clear();
				output.writeShort(Message.MOVE_TO_WAYPOINT);
				output.writeInt(currentWP);
				output.flush();
				message = Arrays.copyOf(outBuffer, output.position());
				cicleTime = System.currentTimeMillis();
				while (wpReachedSemaphore.get() == currentWP) {
					link.sendBroadcastMessage(message);
					
					// Timer
					cicleTime = cicleTime + MUSCOPParam.SENDING_TIMEOUT;
					waitingTime = cicleTime - System.currentTimeMillis();
					if (waitingTime > 0) {
						ardusim.sleep(waitingTime);
					}
				}
				
			} else {
				gui.logVerboseUAV(MUSCOPText.NO_CENTER_WAYPOINT_REACHED_ACK_TALKER);
				output.clear();
				output.writeShort(Message.WAYPOINT_REACHED_ACK);
				output.writeLong(selfId);
				output.writeInt(currentWP);
				output.flush();
				message = Arrays.copyOf(outBuffer, output.position());
				
				cicleTime = System.currentTimeMillis();
				while (wpReachedSemaphore.get() == currentWP) {
					link.sendBroadcastMessage(message);
					
					// Timer
					cicleTime = cicleTime + MUSCOPParam.SENDING_TIMEOUT;
					waitingTime = cicleTime - System.currentTimeMillis();
					if (waitingTime > 0) {
						ardusim.sleep(waitingTime);
					}
				}
			}
			
			/** MOVE_TO_WP PHASE */
			if (currentState.get() == FOLLOWING_MISSION) {
				currentWP++;
				if (listaMasters[indexMasterLocal.get()] == selfId) {
					gui.logVerboseUAV(MUSCOPText.CENTER_SEND_MOVE);
					output.clear();
					output.writeShort(Message.MOVE_TO_WAYPOINT);
					output.writeInt(currentWP);
					output.flush();
					message = Arrays.copyOf(outBuffer, output.position());
					
					cicleTime = System.currentTimeMillis();
					while (moveSemaphore.get() == currentWP) {
						link.sendBroadcastMessage(message);
						
						// Timer
						cicleTime = cicleTime + MUSCOPParam.SENDING_TIMEOUT;
						waitingTime = cicleTime - System.currentTimeMillis();
						if (waitingTime > 0) {
							ardusim.sleep(waitingTime);
						}
					}
				} else {
					gui.logVerboseUAV(MUSCOPText.NO_CENTER_WAYPOINT_REACHED_ACK_TALKER);
					output.clear();
					output.writeShort(Message.WAYPOINT_REACHED_ACK);
					output.writeLong(selfId);
					output.writeInt(currentWP - 1);
					output.flush();
					message = Arrays.copyOf(outBuffer, output.position());
					
					cicleTime = System.currentTimeMillis();
					while (moveSemaphore.get() == currentWP) {
						link.sendBroadcastMessage(message);
						
						// Timer
						cicleTime = cicleTime + MUSCOPParam.SENDING_TIMEOUT;
						waitingTime = cicleTime - System.currentTimeMillis();
						if (waitingTime > 0) {
							ardusim.sleep(waitingTime);
						}
					}
				}
			}
		}
		while (currentState.get() < MOVE_TO_LAND) {
			ardusim.sleep(MUSCOPParam.STATE_CHANGE_TIMEOUT);
		}
		
		/** LANDING PHASE */
		// Only the center UAV gets here
		gui.logVerboseUAV(MUSCOPText.CENTER_SEND_LAND);
		output.clear();
		output.writeShort(Message.LAND);
		Location2DUTM currentLocation = copter.getLocationUTM();
		output.writeDouble(currentLocation.x);
		output.writeDouble(currentLocation.y);
		output.flush();
		message = Arrays.copyOf(outBuffer, output.position());
		
		cicleTime = System.currentTimeMillis();
		while (currentState.get() == LANDING) {
			link.sendBroadcastMessage(message);
			
			// Timer
			cicleTime = cicleTime + MUSCOPParam.SENDING_TIMEOUT;
			waitingTime = cicleTime - System.currentTimeMillis();
			if (waitingTime > 0) {
				ardusim.sleep(waitingTime);
			}
		}
		while (currentState.get() < FINISH) {
			ardusim.sleep(MUSCOPParam.STATE_CHANGE_TIMEOUT);
		}
		
		/** FINISH PHASE */
		gui.logVerboseUAV(MUSCOPText.TALKER_FINISHED);
	}
}
