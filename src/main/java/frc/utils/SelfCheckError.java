package frc.utils;


public class SelfCheckError extends RuntimeException {
    private static final long serialVersionUID = 8749603857922353752L;

    public SelfCheckError(String errorMessage) {
        super(errorMessage);
    }
}