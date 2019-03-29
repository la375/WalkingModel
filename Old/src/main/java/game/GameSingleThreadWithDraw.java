package game;

import org.jbox2d.collision.shapes.*;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;
import org.jbox2d.dynamics.Body;

import java.awt.*;


/**'
 * NOTE: PREFER {@link GameThreadSafe} OVER THIS IMPLEMENTATION FOR LARGE SEARCHES.
 *
 * This creates the QWOP game using the Box2D library. This operates on the primary classloader. This means that
 * multiple instances of this class will interfere with others due to static information inside Box2D.
 * {@link GameThreadSafe} uses a separate classloader for each instance and can be done in multithreaded applications.
 * However, dealing with all the reflection in GameThreadSafe is really annoying. Hence, this class is more readable.
 *
 *
 * @author matt
 */
@SuppressWarnings("Duplicates")
public class GameSingleThreadWithDraw extends GameSingleThread {

    /**
     * Normal stroke for line drawing.
     **/
    private static final Stroke normalStroke = new BasicStroke(0.5f);

    // the other static assignments to avoid null pointers.

    public GameSingleThreadWithDraw() {
        super();
    }

    /**
     * Draw this game's runner. Must provide scaling from game units to pixels, as well as pixel offsets in x and y.
     **/
    public void draw(Graphics g, float scaling, int xOffset, int yOffset) {

        Body newBody = getWorld().getBodyList();
        while (newBody != null) {
            int xOffsetPixels = -(int) (scaling * torsoBody.getPosition().x) + xOffset; // Basic offset, plus centering x on torso.
            Shape newfixture = newBody.getShapeList();

            while (newfixture != null) {

                // Most links are polygon shapes
                if (newfixture.getType() == ShapeType.POLYGON_SHAPE) {

                    PolygonShape newShape = (PolygonShape) newfixture;
                    Vec2[] shapeVerts = newShape.m_vertices;
                    for (int k = 0; k < newShape.m_vertexCount; k++) {

                        XForm xf = newBody.getXForm();
                        Vec2 ptA = XForm.mul(xf, shapeVerts[k]);
                        Vec2 ptB = XForm.mul(xf, shapeVerts[(k + 1) % (newShape.m_vertexCount)]);
                        g.drawLine((int) (scaling * ptA.x) + xOffsetPixels,
                                (int) (scaling * ptA.y) + yOffset,
                                (int) (scaling * ptB.x) + xOffsetPixels,
                                (int) (scaling * ptB.y) + yOffset);
                    }
                } else if (newfixture.getType() == ShapeType.CIRCLE_SHAPE) { // Basically just head
                    CircleShape newShape = (CircleShape) newfixture;
                    float radius = newShape.m_radius;
                    g.drawOval((int) (scaling * (newBody.getPosition().x - radius) + xOffsetPixels),
                            (int) (scaling * (newBody.getPosition().y - radius) + yOffset),
                            (int) (scaling * radius * 2),
                            (int) (scaling * radius * 2));

                } else if (newfixture.getType() == ShapeType.EDGE_SHAPE) { // The track.

                    EdgeShape newShape = (EdgeShape) newfixture;
                    XForm trans = newBody.getXForm();

                    Vec2 ptA = XForm.mul(trans, newShape.getVertex1());
                    Vec2 ptB = XForm.mul(trans, newShape.getVertex2());
                    Vec2 ptC = XForm.mul(trans, newShape.getVertex2());

                    g.drawLine((int) (scaling * ptA.x) + xOffsetPixels,
                            (int) (scaling * ptA.y) + yOffset,
                            (int) (scaling * ptB.x) + xOffsetPixels,
                            (int) (scaling * ptB.y) + yOffset);
                    g.drawLine((int) (scaling * ptA.x) + xOffsetPixels,
                            (int) (scaling * ptA.y) + yOffset,
                            (int) (scaling * ptC.x) + xOffsetPixels,
                            (int) (scaling * ptC.y) + yOffset);

                } else {
                    System.out.println("Not found: " + newfixture.m_type.name());
                }
                newfixture = newfixture.getNext();
            }
            newBody = newBody.getNext();
        }

        //This draws the "road" markings to show that the ground is moving relative to the dude.
        int markingWidth = 5000;
        for (int i = 0; i < markingWidth / 69; i++) {
            g.drawString("_", ((-(int) (scaling * torsoBody.getPosition().x) - i * 70) % markingWidth) + markingWidth, yOffset + 92);
        }
    }

    /**
     * Draw the runner at a specified set of transforms..
     **/
    public static void drawExtraRunner(Graphics2D g, XForm[] transforms, String label, float scaling, int xOffset, int yOffset, Color drawColor, Stroke stroke) {
        g.setColor(drawColor);
        g.drawString(label, xOffset + (int) (transforms[1].position.x * scaling) - 20, yOffset - 75);
        for (int i = 0; i < shapeList.length; i++) {
            g.setColor(drawColor);
            g.setStroke(stroke);
            switch (shapeList[i].getType()) {
                case CIRCLE_SHAPE:
                    CircleShape circleShape = (CircleShape) shapeList[i];
                    float radius = circleShape.getRadius();
                    Vec2 circleCenter = XForm.mul(transforms[i], circleShape.getLocalPosition());
                    g.drawOval((int) (scaling * (circleCenter.x - radius) + xOffset),
                            (int) (scaling * (circleCenter.y - radius) + yOffset),
                            (int) (scaling * radius * 2),
                            (int) (scaling * radius * 2));
                    break;
                case POLYGON_SHAPE:
                    //Get both the shape and its transform.
                    PolygonShape polygonShape = (PolygonShape) shapeList[i];
                    XForm transform = transforms[i];

                    // Ground is black regardless.
                    if (shapeList[i].m_filter.groupIndex == 1) {
                        g.setColor(Color.BLACK);
                        g.setStroke(normalStroke);
                    }
                    for (int j = 0; j < polygonShape.getVertexCount(); j++) { // Loop through polygon vertices and draw lines between them.
                        Vec2 ptA = XForm.mul(transform, polygonShape.m_vertices[j]);
                        Vec2 ptB = XForm.mul(transform, polygonShape.m_vertices[(j + 1) % (polygonShape.getVertexCount())]); //Makes sure that the last vertex is connected to the first one.
                        g.drawLine((int) (scaling * ptA.x) + xOffset,
                                (int) (scaling * ptA.y) + yOffset,
                                (int) (scaling * ptB.x) + xOffset,
                                (int) (scaling * ptB.y) + yOffset);
                    }
                    break;
                default:
                    break;
            }
        }
    }
}
