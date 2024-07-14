package friarLib2.Logging;

/**
 * Props to Team 3256 for this documentation of logging as I (Zach)
 * have no clue otherwise
 */

/**
 * Simple yet moderately fast I/O routines.
 *
 * <p>Example usage:
 *
 * <p>Kattio io = new Kattio(System.in, System.out);
 *
 * <p>while (io.hasMoreTokens()) { int n = io.getInt(); double d = io.getDouble(); double ans = d*n;
 *
 * <p>io.println("Answer: " + ans); }
 *
 * <p>io.close();
 *
 * <p>
 *
 * <p>Some notes:
 *
 * <p>- When done, you should always do io.close() or io.flush() on the Kattio-instance, otherwise,
 * you may lose output.
 *
 * <p>- The getInt(), getDouble(), and getLong() methods will throw an exception if there is no more
 * data in the input, so it is generally a good idea to use hasMoreTokens() to check for
 * end-of-file.
 */

import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.util.StringTokenizer;

public class Kattio extends PrintWriter {

    private BufferedReader reader;
    private String line;
    private StringTokenizer stringTokenizer;
    private String token;

    public Kattio(InputStream i)
    {
        super(new BufferedOutputStream(System.out));
        reader = new BufferedReader(new InputStreamReader(i));
    }

    public Kattio(InputStream i, OutputStream o)
    {
        super(new BufferedOutputStream(o));
        reader = new BufferedReader(new InputStreamReader(i));
    }

    public boolean hasMoreTokens()
    {
        return peekToken() != null;
    }

    public int getInt()
    {
        return Integer.parseInt(nextToken());
    }

    public double getDouble()
    {
        return Double.parseDouble(nextToken());
    }

    public long getLong()
    {
        return Long.parseLong(nextToken());
    }

    public String getWord()
    {
        return nextToken();
    }

    private String peekToken()
    {
        if (token == null)
            try {
                while (stringTokenizer == null || !stringTokenizer.hasMoreTokens())
                {
                    line = reader.readLine();
                    if (line == null) return null;
                    stringTokenizer = new StringTokenizer(line);
                }
                token = stringTokenizer.nextToken();
            } catch (IOException ignored) {}
        return token;
    }

    private String nextToken()
    {
        String ans = peekToken();
        token = null;
        return ans;
    }


}
