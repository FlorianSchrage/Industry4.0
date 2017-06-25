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
	
	public SenderThread(Robot apiRef, Semaphore sema)
	{
		this.apiRef = apiRef;
		this.sema = sema;
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
				sema.acquire();
			} catch (InterruptedException e)
			{
				e.printStackTrace();
			}
			
			apiRef.print("Sender aquired");
			Socket newSocket = apiRef.getSocket();

			try
			{
				if(newSocket != socket)
				{
					apiRef.print("Sender new Socket");
					socket = newSocket;
					networkOut = new PrintWriter(socket.getOutputStream());
				}

				apiRef.print("Sending " + apiRef.getJsonString());
				networkOut.println(apiRef.getJsonString());
				networkOut.flush();

			} catch(IOException e)
			{
				e.printStackTrace();
			}
			finally
			{
				sema.release();
			}
			
			try
			{
				Thread.sleep(1000);
			} catch (InterruptedException e)
			{
				e.printStackTrace();
			}
		}
	}
}
