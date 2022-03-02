package org.firstinspires.ftc.teamcode.structures;

public class TelemetryObj<T> {
    private String caption;
    private T content;
    public TelemetryObj(String caption, T content) {
        this.caption = caption;
        this.content = content;
    }
    public TelemetryObj(String caption) {
        this.caption = caption;
    }

    public String getCaption() {
        return caption;
    }

    public T getContent() {
        return content;
    }

    public void setContent(T content) {
        this.content = content;
    }
}
