import { MiGPT } from "mi-gpt";
import type { GlobalConfig } from "./config";

/**
 * createMiGPTClient
 * @param appConfig 
 * @returns a working MiGPT instance. 
 * @description This function creates a new MiGPT client with the provided configuration.
 * It uses the `MiGPT.create` method to initialize the client and returns it. 
 * The speaker configuration is spread from the appConfig object into the MiGPT configuration. 
 * You need to execute .start() method to start the background service.
 */

export const createMiGPTClient = async (appConfig: GlobalConfig) => {
	MiGPT.reset();
	return MiGPT.create({
		speaker: {
			...appConfig.speaker,
		},
		systemTemplate: appConfig.systemTemplate,
		bot: appConfig.bot,
		master: appConfig.master,
		room: appConfig.room,
	});
}
