package friarLib2.math;

import java.awt.*;
import java.awt.geom.Point2D;
import java.sql.Array;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.concurrent.locks.Condition;

public class LookupTable {

    private ArrayList<Point2D.Double> Nodes = new ArrayList<Point2D.Double>();

    public LookupTable AddValue(double input, double output)
    {
        Nodes.add(new Point2D.Double(input, output));
        Nodes.sort(Comparator.comparingDouble(a -> a.x));

        return this;
    }

    public double GetValue(double input){

        assert Nodes.size() > 1 : "LookupTable requires at least two values to lerp between";

        var numModes = Nodes.size();

        // -- Clamp low
        var low = Nodes.get(0);
        if (input <= low.x)
        {
            return low.y;
        }

        // -- Clamp High
        var high = Nodes.get(numModes - 1);
        if (input >= high.x)
        {
            return high.y;
        }

        Point2D.Double Left = Nodes.get(0);
        Point2D.Double Right = null;

        // -- Find node we're past
        for (Point2D.Double Node : Nodes)
        {
            if (input > Node.x)
            {
                Left = Node;
            }
            else
            {
                Right = Node;
                break;
            }
        }

        assert Right != null : "Didn't find a Right value";

        // -- Early out in outputs are same to avoid division by 0
        if (Left.y == Right.y)
        {
            return Left.y;
        }

        return FriarMath.Remap(input, Left.x, Right.x, Left.y, Right.y);
    }
}
