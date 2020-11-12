import java.io.*;
import java.net.*;

public class Remote_Connection {
	public static void main(String[] args) throws IOException {
		String ip = "10.0.1.1"; // BT
		Socket sock = new Socket(ip, 2554);
		System.out.println("Connected");
		InputStream in = sock.getInputStream();
		DataInputStream dIn = new DataInputStream(in);
		
		while (dIn.readBoolean()) {
			for(int i = 0; i < 19; i++) {
				for(int j = 0; j < 19; j++) {
					char next = dIn.readChar();
					System.out.print(next + " ");
				}
				System.out.println();	
			}
			System.out.println();
		}
		sock.close();
	}
}