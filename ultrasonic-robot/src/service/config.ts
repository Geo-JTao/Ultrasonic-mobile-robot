import path from "node:path";
import * as fs from "node:fs";

interface Bot {
    name: string;
    profile: string;
}

interface Master {
    name: string;
    profile: string;
}

interface Room {
    name: string;
    description: string;
}

interface Speaker {
    userId: string;
    password: string;
    did: string;
    callAIKeywords: string[];
    wakeUpKeywords: string[];
    exitKeywords: string[];
    onEnterAI: string[];
    onExitAI: string[];
    onAIAsking: string[];
    onAIReplied: string[];
    onAIError: string[];
    ttsCommand: [number, number];
    wakeUpCommand: [number, number];
    tts: "xiaoai" | "custom";
    streamResponse: boolean;
    exitKeepAliveAfter: number;
    checkTTSStatusAfter: number;
    checkInterval: number;
    debug: boolean;
    enableTrace: boolean;
    timeout: number;
}

interface Config {
    systemTemplate: string;
    bot: Bot;
    master: Master;
    room: Room;
    speaker: Speaker;
}

interface AppConfig {
    port: number;
}

export interface GlobalConfig extends Config {
    app: AppConfig;
}

const defaultConfig: Config = {
    systemTemplate: "",
    bot: {
        name: "",
        profile: "",
    },
    master: {
        name: "",
        profile: "",
    },
    room: {
        name: "",
        description: "",
    },
    speaker: {
        userId: "",
        password: "",
        did: "小爱音箱Pro",
        callAIKeywords: [],
        wakeUpKeywords: [],
        exitKeywords: [],
        onEnterAI: [],
        onExitAI: [],
        onAIAsking: [],
        onAIReplied: [],
        onAIError: [],
        ttsCommand: [5, 1],
        wakeUpCommand: [5, 3],
        tts: "xiaoai",
        streamResponse: true,
        exitKeepAliveAfter: 30,
        checkTTSStatusAfter: 3,
        checkInterval: 1500,
        debug: false,
        enableTrace: false,
        timeout: 5000,
    },
};

/**
 * createAppConfig
 * @returns GlobalConfig object with the configuration loaded from environment variables and a default configuration.
 * @description This function reads the configuration from environment variables and a JSON file and merges it with default values.
 * It returns a `GlobalConfig` object that includes the application configuration and the merged configuration.
 * By default it reads from config.json
 */
export const createAppConfig = () => {
    let config: Config = defaultConfig;

    const configFile = fs.readFileSync(
        path.resolve(process.cwd(), "./config.json"),
        "utf8",
    );

    config = JSON.parse(configFile) as Config;

    config.speaker.userId = process.env.SPEAKER_USERID || config.speaker.userId;
    config.speaker.password = process.env.SPEAKER_USER_PASSWORD || config.speaker.password;
    config.speaker.did = process.env.SPEAKER_DID || config.speaker.did;

    return {
        app: {
            port: Number.parseInt(process.env.PORT || "3000", 10),
        },
        ...config,
    } as GlobalConfig;
};
