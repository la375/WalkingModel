import game.GameSingleThreadWithDraw;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;

/**
 * This is a playable QWOP game. Press Q, W, O, P keys to move. R resets the runner to its original state.
 * This is meant to be a standalone example of using the interface to the game.
 *
 * @author matt
 */
public class PlayableQWOP extends JPanel implements KeyListener, ActionListener {

    /**
     * Keep track of which keys are down. True means currently pressed.
     */
    private boolean q, w, o, p;

    /**
     * Game physics world to use.
     */
    private GameSingleThreadWithDraw game = new GameSingleThreadWithDraw();

    @Override
    public void actionPerformed(ActionEvent e) { // Gets called every 40ms
        game.stepGame(q, w, o, p); // Step the game forward 1 timestep with the specified keys pressed.
        repaint(); // Redraw the runner and scene.
    }

    @Override
    public void paint(Graphics g) {
        super.paint(g);
        game.draw(g, 10, 300, 200); // Redraws the game. Scaling and offsets are handpicked to work for the size of
        // the window.
    }

    @Override
    public void keyPressed(KeyEvent event) {
        switch (event.getKeyCode()) {
            case KeyEvent.VK_Q:
                q = true;
                break;
            case KeyEvent.VK_W:
                w = true;
                break;
            case KeyEvent.VK_O:
                o = true;
                break;
            case KeyEvent.VK_P:
                p = true;
                break;
            case KeyEvent.VK_R: // Reset the runner on pressing r.
                game.makeNewWorld();
                break;
        }
    }

    @Override
    public void keyReleased(KeyEvent event) {
        switch (event.getKeyCode()) {
            case KeyEvent.VK_Q:
                q = false;
                break;
            case KeyEvent.VK_W:
                w = false;
                break;
            case KeyEvent.VK_O:
                o = false;
                break;
            case KeyEvent.VK_P:
                p = false;
                break;
        }
    }

    @Override
    public void keyTyped(KeyEvent e) {}

    public static void main(String[] args) {
        // JPanel that runs and displays the game.
        PlayableQWOP qwop = new PlayableQWOP();

        JFrame frame = new JFrame(); // New frame to hold and manage the QWOP JPanel.
        frame.add(qwop);
        frame.addKeyListener(qwop); // Listen for user input.
        frame.setPreferredSize(new Dimension(600, 400));
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.pack();
        frame.setVisible(true);

        // Fire game update every 40 ms.
        new Timer(40, qwop).start();
    }
}
