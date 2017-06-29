import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;
import java.util.concurrent.Semaphore;

public class SenderThread extends Thread
{
	private volatile boolean running = true;
	private Robot apiRef;
	private Semaphore sema;
	private Socket socket;
	private PrintWriter networkOut;
	private static final int PORT = 22222;
	private boolean firstTime;
	private int delay_initialize;
	private int delay_sending;
	
	public SenderThread(Robot apiRef, Semaphore sema, int delay_initialize, int delay_sending)
	{
		this.apiRef = apiRef;
		this.sema = sema;
		this.delay_initialize = delay_initialize;
		this.delay_sending = delay_sending;
		firstTime = true;
	}
	
	public void terminate()
	{
		running = false;
	}
	
	public void run()
	{
		while(running)
		{
			try
			{
				if(firstTime) {
					try
					{
						Thread.sleep(delay_initialize);
					} catch (InterruptedException e)
					{
						e.printStackTrace();
					}
				}
				firstTime = false;
				
				apiRef.print("Sender starting");
				socket = new Socket(Robot.ip_host, PORT);
				networkOut = new PrintWriter(socket.getOutputStream());
				apiRef.print("Socket (S) opened");
				apiRef.print("IP: " + socket.getLocalAddress().getHostAddress());
			
				try
				{
					sema.acquire();
				} catch (InterruptedException e)
				{
					e.printStackTrace();
				}
				String jsonData = apiRef.getJsonString();
				sema.release();
				
				apiRef.print("Sending " + jsonData);
				networkOut.println(jsonData);
				networkOut.flush();
				sema.release();

			} catch(IOException e)
			{
				e.printStackTrace();
			}
			finally
			{
				try{ socket.close(); } catch(Exception e) {}
				try{ networkOut.close(); } catch(Exception e) {}
			}
			
			try
			{
				Thread.sleep(delay_sending);
			} catch (InterruptedException e)
			{
				e.printStackTrace();
			}
		}
	}
	
	public Socket getSocket() {
		return socket;
	}
}
