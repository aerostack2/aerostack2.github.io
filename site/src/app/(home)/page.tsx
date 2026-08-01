import Link from "next/link";
import { appDescription, appName, gitConfig } from "@/lib/shared";

const features = [
  {
    title: "Teleoperation to autonomy",
    body: "Fly manually, or build systems that carry out aerial missions without an operator.",
  },
  {
    title: "Single robot or swarm",
    body: "Validated flying several heterogeneous drones at once, indoors and outdoors.",
  },
  {
    title: "Hardware independent",
    body: "Runs on laptops and onboard computers, across DJI, Pixhawk, Crazyflie and more.",
  },
  {
    title: "Fully modular",
    body: "A plugin architecture lets every component be swapped without touching the rest.",
  },
];

export default function HomePage() {
  return (
    <main className="flex flex-1 flex-col">
      <section className="mx-auto flex w-full max-w-4xl flex-col items-center px-4 py-24 text-center">
        <h1 className="text-4xl font-semibold tracking-tight sm:text-5xl">
          {appName}
        </h1>
        <p className="mt-5 max-w-2xl text-lg text-fd-muted-foreground">
          {appDescription}
        </p>
        <div className="mt-8 flex flex-wrap justify-center gap-3">
          <Link
            href="/docs/getting-started"
            className="rounded-lg bg-fd-primary px-5 py-2.5 text-sm font-medium text-fd-primary-foreground transition-opacity hover:opacity-90"
          >
            Get started
          </Link>
          <Link
            href="/docs"
            className="rounded-lg border px-5 py-2.5 text-sm font-medium transition-colors hover:bg-fd-accent"
          >
            Documentation
          </Link>
          <a
            href={`https://github.com/${gitConfig.user}/aerostack2`}
            className="rounded-lg border px-5 py-2.5 text-sm font-medium transition-colors hover:bg-fd-accent"
          >
            GitHub
          </a>
        </div>
      </section>

      <section className="mx-auto grid w-full max-w-5xl gap-4 px-4 pb-24 sm:grid-cols-2">
        {features.map((feature) => (
          <div key={feature.title} className="rounded-xl border bg-fd-card p-6">
            <h2 className="font-medium">{feature.title}</h2>
            <p className="mt-2 text-sm text-fd-muted-foreground">
              {feature.body}
            </p>
          </div>
        ))}
      </section>
    </main>
  );
}
