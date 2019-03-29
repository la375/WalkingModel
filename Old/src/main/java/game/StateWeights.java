package game;

public class StateWeights {

    private static final float W_BODY = 1f, W_HEAD = 1.2f, W_RTHIGH = 1f, W_LTHIGH = 1f,
            W_RCALF = 1f, W_LCALF = 1f, W_RFOOT = 1.2f, W_LFOOT = 1.2f,
            W_RUARM = 1f, W_LUARM = 1f, W_RLARM = 0.8f, W_LLARM = 0.8f;

    private static final float W_X = 1f, W_Y = 1f, W_TH = 1f,
            W_DX = 1f, W_DY = 1f, W_DTH = 1f;

    public static float getWeight(State.ObjectName obj, State.StateName st) {
        float multiplier = 1f;
        switch (obj) {
            case BODY:
                multiplier *= W_BODY;
                break;
            case HEAD:
                multiplier *= W_HEAD;
                break;
            case LCALF:
                multiplier *= W_LCALF;
                break;
            case LFOOT:
                multiplier *= W_LFOOT;
                break;
            case LLARM:
                multiplier *= W_LLARM;
                break;
            case LTHIGH:
                multiplier *= W_LTHIGH;
                break;
            case LUARM:
                multiplier *= W_LUARM;
                break;
            case RCALF:
                multiplier *= W_RCALF;
                break;
            case RFOOT:
                multiplier *= W_RFOOT;
                break;
            case RLARM:
                multiplier *= W_RLARM;
                break;
            case RTHIGH:
                multiplier *= W_RTHIGH;
                break;
            case RUARM:
                multiplier *= W_RUARM;
                break;
            default:
                break;
        }

        switch (st) {
            case X:
                multiplier *= W_X;
                break;
            case Y:
                multiplier *= W_Y;
                break;
            case TH:
                multiplier *= W_TH;
                break;
            case DX:
                multiplier *= W_DX;
                break;
            case DY:
                multiplier *= W_DY;
                break;
            case DTH:
                multiplier *= W_DTH;
                break;
        }
        return multiplier;
    }
}
