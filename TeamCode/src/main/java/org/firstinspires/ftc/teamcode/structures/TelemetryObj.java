package org.firstinspires.ftc.teamcode.structures;

public class TelemetryObj {
    private String caption;
    private Object content;
    public TelemetryObj(String caption, Object content) {
        this.caption = caption;
        this.content = content;
    }
    public TelemetryObj(String caption) {
        this.caption = caption;
    }

    public String getCaption() {
        return caption;
    }

    public Object getContent() {
        return content;
    }

    public void setContent(Object content) {
        this.content = content;
    }
}
