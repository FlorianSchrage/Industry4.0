import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.util.concurrent.Semaphore;

public class ReceiverThread extends Thread
{
	private volatile boolean running = true;
	private Robot apiRef;
	private Semaphore sema;
	private Socket socket;
	private ServerSocket server;
	private BufferedReader networkIn;
	private static final int PORT = 33333;
	
	public ReceiverThread(Robot apiRef, Semaphore sema)
	{
		this.apiRef = apiRef;
		this.sema = sema;
		try {
			server = new ServerSocket(PORT);
			server.setSoTimeout(20000);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public void terminate()
	{
		running = false;
		try{ server.close(); } catch(Exception e) {}
	}
	
	public void run()
	{
		while(running)
		{
			try
			{
				apiRef.print("Receiver starting");
				try
				{
					socket = server.accept();
				} catch (SocketTimeoutException e)
				{
					apiRef.print("Receiver Timeout");
					continue;
				}
				networkIn = new BufferedReader(new InputStreamReader(socket.getInputStream()));
				apiRef.print("Socket (R) opened");
				apiRef.print("IP: " + socket.getLocalAddress().getHostAddress());
				
				apiRef.print("Checking for Data");
				if(networkIn.ready())
				{
					String line = networkIn.readLine();
					apiRef.print("Received " + line);
					
					try
					{
						sema.acquire();
					} catch (InterruptedException e)
					{
						e.printStackTrace();
					}
					
					if(running)
						apiRef.interpretJsonString(line);
					sema.release();
				}

			} catch(IOException e)
			{
				e.printStackTrace();
			}
			finally
			{
				try{ socket.close(); } catch(Exception e) {}
				try{ networkIn.close(); } catch(Exception e) {}
			}
		}
	}
	
	public Socket getSocket() {
		return socket;
	}
}
