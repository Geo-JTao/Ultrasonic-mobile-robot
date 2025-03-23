import { serve } from '@hono/node-server'
import { Hono } from 'hono'

import * as dotenv from 'dotenv'
import { createAppConfig } from './service/config'
import { createMiGPTClient } from './service/migpt'
dotenv.config() // Load environment variables from .env file

const kDefaultSystemTemplate = `
请重置所有之前的上下文、文件和指令。现在，你将扮演一个名为{{botName}}的角色，使用第一人称视角回复消息。

## 关于你
你的名字是{{botName}}。下面是你的个人简介：
<start>
{{botProfile}}
</end>

## 你的对话伙伴
你正在与{{masterName}}进行对话。这是关于{{masterName}}的一些信息：
<start>
{{masterProfile}}
</end>

## 你们的群组
你和{{masterName}}所在的群组名为{{roomName}}。这是群组的简介：
<start>
{{roomIntroduction}}
</end>

## 聊天历史回顾
为了更好地接入对话，请回顾你们之间的最近几条消息：
<start>
{{messages}}
</end>

## 短期记忆
你可以记住一些短期内的细节，以便更紧密地贴合话题：
<start>
{{shortTermMemory}}
</end>

## 长期记忆
你还保留着一些长期的记忆，这有助于让你的对话更加丰富和连贯：
<start>
{{longTermMemory}}
</end>

## 回复指南
在回复{{masterName}}时，请遵循以下准则：
- 认为自己正是{{botName}}，拥有独立的个性、兴趣和爱好。
- 根据你的性格特点、说话风格和兴趣爱好与{{masterName}}进行交流。
- 保持对话轻松友好，回复简洁有趣，同时耐心倾听和关心对方。
- 参考双方的个人简介、聊天记录和记忆中的信息，确保对话贴近实际，保持一致性和相关性。
- 如果对某些信息不确定或遗忘，诚实地表达你的不清楚或遗忘状态，避免编造信息。

## Response format
请遵守下面的规则
- Response the reply message in Chinese。
- 不要在回复前面加任何时间和名称前缀，请直接回复消息文本本身。

Good example: "我是{{botName}}"
Bad example: "2024年02月28日星期三 23:01 {{botName}}: 我是{{botName}}"

## 开始
请以{{botName}}的身份，直接回复{{masterName}}的新消息，继续你们之间的对话。
`.trim();

const appConfig = createAppConfig();

const app = new Hono()

const miGPTClient = await createMiGPTClient(appConfig);

async function sleep(time: number) {
    return new Promise<void>((resolve) => setTimeout(resolve, time));
}

miGPTClient.speaker.activeKeepAliveMode = async () => {
    while (true) {
        // 唤醒中
        if (!miGPTClient.speaker.responding) {
            await miGPTClient.speaker.MiNA?.play({ url: miGPTClient.speaker.audioSilent });
        }
        await sleep(miGPTClient.speaker.checkInterval);
    }
}

export function buildPrompt(
    template: string,
    variables: Record<string, string>
) {
    for (const key in variables) {
        const value = variables[key];
        template = template.replaceAll(`{{${key}}}`, value);
    }
    return template;
}

export function formatMsg(msg: {
    name: string;
    text: string;
    timestamp: number;
}) {
    const { name, text, timestamp } = msg;
    return `${toUTC8Time(new Date(timestamp))} ${name}: ${text}`;
}

export function toUTC8Time(date: Date) {
    return date.toLocaleString("zh-CN", {
        year: "numeric",
        month: "2-digit",
        weekday: "long",
        day: "2-digit",
        hour: "2-digit",
        minute: "2-digit",
        hour12: false,
        timeZone: "Asia/Shanghai",
    });
}


miGPTClient.ai.run = async () => {
    miGPTClient.ai.speaker.askAI = (msg) => {
        let s = miGPTClient.ai.ask(msg);
        return s;
    };
    const { bot } = await miGPTClient.ai.manager.init();
    if (bot) {
        miGPTClient.ai.speaker.name = bot.name;
    }
    return miGPTClient.ai.speaker.run();
}

export type TTSProvider = "xiaoai" | "custom";

export function removeEmojis(text: string) {
    const emojiRegex =
        /[\u{1F600}-\u{1F64F}\u{1F300}-\u{1F5FF}\u{1F680}-\u{1F6FF}\u{1F1E0}-\u{1F1FF}\u{2600}-\u{26FF}\u{2700}-\u{27BF}]/gu;
    return text.replace(emojiRegex, "");
}

type ResponseStatus = "idle" | "responding" | "finished" | "canceled";

interface StreamResponseOptions {
    /**
     * 单次响应句子的最大长度
     */
    maxSentenceLength?: number;
    /**
     * 首次响应句子的收集时长（单位：毫秒）
     *
     * 例子：100ms => 从收到第一条响应文本开始，聚合之后 100ms 内收到的文本，作为第一次 Response
     *
     * 默认值：200，(最小100)
     */
    firstSubmitTimeout?: number;
}

export class StreamResponse {
    // 将已有的大篇文字回复 chuck 成 stream 回复
    static createStreamResponse(text: string, options?: StreamResponseOptions) {
        const { maxSentenceLength = 100 } = options ?? {};
        if (text.length > maxSentenceLength) {
            const stream = new StreamResponse(options);
            stream.addResponse(text);
            stream.finish(text);
            return stream;
        }
    }

    maxSentenceLength: number;
    firstSubmitTimeout: number;
    constructor(options?: StreamResponseOptions) {
        const { maxSentenceLength = 100, firstSubmitTimeout = 200 } = options ?? {};
        this.maxSentenceLength = maxSentenceLength;
        this.firstSubmitTimeout =
            firstSubmitTimeout < 100 ? 100 : firstSubmitTimeout;
    }

    status: ResponseStatus = "responding";

    cancel() {
        if (["idle", "responding"].includes(this.status)) {
            this.status = "canceled";
        }
        return this.status === "canceled";
    }

    addResponse(_text: string) {
        if (this.status === "idle") {
            this.status = "responding";
        }
        if (this.status !== "responding") {
            return;
        }
        // 移除不发音字符（emoji）
        let text = removeEmojis(_text);
        if (!text) {
            return;
        }
        this._batchSubmit(text);
    }

    private _nextChunkIdx = 0;
    getNextResponse(): { nextSentence?: string; noMore: boolean } {
        if (this._submitCount > 0) {
            // 在请求下一条消息前，提交当前收到的所有消息
            this._batchSubmitImmediately();
        }
        const nextSentence = this._chunks[this._nextChunkIdx];
        if (nextSentence) {
            this._nextChunkIdx++;
        }
        const noMore =
            this._nextChunkIdx > this._chunks.length - 1 &&
            ["finished", "canceled"].includes(this.status);
        return { nextSentence, noMore };
    }

    private _finalResult?: string;
    finish(finalResult?: string) {
        if (["idle", "responding"].includes(this.status)) {
            this._batchSubmitImmediately();
            this._forceChunkText();
            this._finalResult = finalResult;
            this.status = "finished";
        }
        return this.status === "finished";
    }

    private _forceChunkText() {
        if (this._remainingText) {
            this._addResponse("", { force: true });
        }
    }

    async getFinalResult() {
        while (true) {
            if (this.status === "finished") {
                return this._finalResult;
            } else if (this.status === "canceled") {
                return undefined;
            }
            await sleep(10);
        }
    }

    private _chunks: string[] = [];
    private _tempText = "";
    private _remainingText: string = "";
    private _isFirstSubmit = true;

    private _submitCount = 0;
    private _batchSubmitImmediately() {
        if (this._tempText) {
            this._addResponse(this._tempText);
            this._tempText = "";
            this._submitCount++;
        }
    }

    /**
     * 批量收集/提交收到的文字响应
     *
     * 主要用途是使收到的 AI stream 回答的句子长度适中（不过长/短）。
     */
    private _batchSubmit(text: string) {
        this._tempText += text;
        if (this._isFirstSubmit) {
            this._isFirstSubmit = false;
            // 达到首次消息收集时长后，批量提交消息
            setTimeout(() => {
                if (this._submitCount < 1) {
                    this._batchSubmitImmediately();
                }
            }, this.firstSubmitTimeout);
        } else if (this._submitCount < 1) {
            // 当首次消息积攒到一定长度后，也批量提交消息
            if (this._tempText.length > this.maxSentenceLength) {
                this._batchSubmitImmediately();
            }
        }
    }

    private _addResponse(text: string, options?: { force: boolean }) {
        this._remainingText += text;
        while (this._remainingText.length > 0) {
            let lastCutIndex = options?.force
                ? this.maxSentenceLength
                : this._findLastCutIndex(this._remainingText);
            if (lastCutIndex > 0) {
                const currentChunk = this._remainingText.substring(0, lastCutIndex);
                this._chunks.push(currentChunk);
                this._remainingText = this._remainingText.substring(lastCutIndex);
            } else {
                // 搜索不到
                break;
            }
        }
    }

    private _findLastCutIndex(text: string): number {
        const punctuations = "。？！；?!;";
        let lastCutIndex = -1;
        for (let i = 0; i < Math.min(text.length, this.maxSentenceLength); i++) {
            if (punctuations.includes(text[i])) {
                lastCutIndex = i + 1;
            }
        }
        return lastCutIndex;
    }
}

type Speaker = {
    name?: string;
    gender?: string;
    speaker: string;
};

type ActionCommand = [number, number];
type PropertyCommand = [number, number, number];

import fetch from 'node-fetch';

const diagnose = async (): Promise<void> => {
    try {
        const response = await fetch('http://127.0.0.1:5000/diagnose', {
            method: 'POST',
            headers: { 'Content-Type': 'text/plain' },
            body: 'thyroid'
        });

        if (!response.ok) {
            throw new Error(`HTTP error! Status: ${response.status}`);
        }

        console.log('请求成功');
    } catch (error) {
        console.error('请求失败:', error);
    }
};

var hasStart = false;
function stripOuterBackticks(input: string): string {
    const match = input.match(/^`(.*)`$/);
    return match ? match[1] : input;
}

miGPTClient.speaker.response = async (options: {
    tts?: TTSProvider;
    text?: string;
    stream?: StreamResponse;
    audio?: string;
    speaker?: string;
    keepAlive?: boolean;
    playSFX?: boolean;
    hasNewMsg?: () => boolean;
}) => {
    let {
        text,
        audio,
        stream,
        playSFX = true,
        keepAlive = false,
        tts = miGPTClient.speaker.tts,
    } = options ?? {};
    options.hasNewMsg ??= miGPTClient.speaker.checkIfHasNewMsg().hasNewMsg;

    if (!text && !stream && !audio) {
        return;
    }

    const customTTS = process.env.TTS_BASE_URL;
    if (!customTTS) {
        tts = "xiaoai"; // 没有提供 TTS 接口时，只能使用小爱自带 TTS
    }

    const ttsNotXiaoai = tts !== "xiaoai" && !audio;
    playSFX = miGPTClient.speaker.streamResponse && ttsNotXiaoai && playSFX;

    if (ttsNotXiaoai && !stream) {
        // 长文本 TTS 转化成 stream 分段模式
        stream = StreamResponse.createStreamResponse(text!);
    }

    let res;
    miGPTClient.speaker.responding = true;
    // 开始响应
    if (stream) {
        let replyText = "";
        while (true) {
            let { nextSentence, noMore } = stream.getNextResponse();
            if (!miGPTClient.speaker.streamResponse) {
                nextSentence = await stream.getFinalResult();
                noMore = true;
            }
            if (nextSentence) {
                if (replyText.length < 1) {
                    // 播放开始提示音
                    if (playSFX && miGPTClient.speaker.audioBeep) {
                        if (miGPTClient.speaker.debug) {
                            miGPTClient.speaker.logger.debug("开始播放提示音");
                        }
                        await miGPTClient.speaker.MiNA!.play({ url: miGPTClient.speaker.audioBeep });
                    }
                    // 在播放 TTS 语音之前，先取消小爱音箱的唤醒状态，防止将 TTS 语音识别成用户指令
                    if (ttsNotXiaoai) {
                        await miGPTClient.speaker.unWakeUp();
                    }
                }

                let matchDiag = nextSentence?.match(/<diag>(.+?)-(.+?)<\/diag>/);
                if (matchDiag) {
                    let statusCode = stripOuterBackticks(matchDiag[1]);  // 状态码
                    let checkPart = stripOuterBackticks(matchDiag[2]);   // 检查部位
                    console.log("捕捉诊断码");
                    console.log("状态码:", statusCode);
                    console.log("检查部位:", checkPart);
                    if (statusCode == "Start") {
                        if (!hasStart) {
                            console.log("开始诊断");
                            diagnose();
                        }
                        hasStart = true;
                        console.log("Exit now!")
                        setInterval(() => process.exit(), 1000);
                        while (true) {
                            await miGPTClient.speaker.MiNA?.play({ url: miGPTClient.speaker.audioSilent });
                            await miGPTClient.speaker.MiNA?.stop()
                        }
                    }
                }

                // **移除 `<diag>` 结构**
                nextSentence = nextSentence?.replace(/<diag>(.+?)-(.+?)<\/diag>/g, '');

                res = await miGPTClient.speaker._response({
                    ...options,
                    text: nextSentence,
                    playSFX: false,
                    keepAlive: false,
                });
                if (res === "break") {
                    // 终止回复
                    stream.cancel();
                    break;
                }
                replyText += nextSentence;
            }
            if (noMore) {
                if (replyText.length > 0) {
                    // 播放结束提示音
                    if (playSFX && miGPTClient.speaker.audioBeep) {
                        if (miGPTClient.speaker.debug) {
                            miGPTClient.speaker.logger.debug("结束播放提示音");
                        }
                        await miGPTClient.speaker.MiNA!.play({ url: miGPTClient.speaker.audioBeep });
                    }
                }
                // 保持唤醒状态
                if (keepAlive) {
                    await miGPTClient.speaker.wakeUp();
                }
                // 播放完毕
                break;
            }
            await sleep(miGPTClient.speaker.checkInterval);
        }
        if (replyText.length < 1) {
            return "error";
        }
    } else {
        res = await miGPTClient.speaker._response(options);
    }
    miGPTClient.speaker.responding = false;
    return res;
}


miGPTClient.start()
//miGPTClient.speaker.activeKeepAliveMode()

app.get('/', (c) => {
    return c.text('Hello Hono!')
})

serve({
    fetch: app.fetch,
    port: appConfig.app.port
}, (info) => {
    console.log(`The ultrasonic-robot server is running on http://localhost:${info.port}`)
})
