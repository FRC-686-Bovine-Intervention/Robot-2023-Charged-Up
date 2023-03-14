package frc.robot.auto;

import org.littletonrobotics.junction.Logger;

public class AutoConfiguration {
    public enum StartPosition {
        Wall,
        Center,
        Loading
    }
    public enum GamePiece {
        Cone,
        Cube
    }
    public final StartPosition startingPosition;
    public final GamePiece startingPiece;
    public final GamePiece[] stagedPieces;

    public AutoConfiguration() {
        this(StartPosition.Wall);
    }
    public AutoConfiguration(StartPosition startingPosition) {
        this(startingPosition, GamePiece.Cone);
    }
    public AutoConfiguration(StartPosition startingPosition, GamePiece startingPiece) {
        this(startingPosition, startingPiece, new GamePiece[]{GamePiece.Cone,GamePiece.Cone,GamePiece.Cone,GamePiece.Cone});
    }
    public AutoConfiguration(StartPosition startingPosition, GamePiece startingPiece, GamePiece[] stagedPieces) {
        this.startingPosition = startingPosition;
        this.startingPiece = startingPiece;
        this.stagedPieces = stagedPieces;
    }

    public AutoConfiguration log(Logger logger, String prefix) {
        String[] stagedNames = new String[stagedPieces.length];
        for(int i = 0; i < stagedNames.length; i++) {
            stagedNames[i] = stagedPieces[i] != null ? stagedPieces[i].name() : "null";
        }
        logger.recordOutput(prefix + "/Starting Position", startingPosition != null ? startingPosition.name() : "null");
        logger.recordOutput(prefix + "/Starting Piece", startingPiece != null ? startingPiece.name() : "null");
        logger.recordOutput(prefix + "/Staged Pieces", stagedNames);
        return this;
    }
}
