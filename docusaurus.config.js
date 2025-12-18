// @ts-check
// `@type` JSDoc annotations allow better IDE autocompletion and type checking
// (when paired with `@ts-check`).
// There are two ways to use this export and have Docusaurus automatically pick it up:
// 1. Explicitly return it with the default export
// 2. It implicitly becomes the default export

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'The Robotic Nervous System: ROS 2 for Humanoid AI',
  tagline: 'Teaching ROS 2 middleware for AI agents controlling humanoid robots',
  favicon: 'img/favicon.ico',
  url: 'https://tahirrasheed.github.io',
  baseUrl: '/hackathon-book/',
  organizationName: 'TahirRasheed',
  projectName: 'hackathon-book',
  onBrokenLinks: 'warn',
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
      },
    },
  },
  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          editUrl: 'https://github.com/anthropics/hackathon-book/tree/main/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'ROS 2 Nervous System',
        logo: {
          alt: 'ROS 2 Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Modules',
          },
          {
            type: 'localeDropdown',
            position: 'right',
          },
          {
            href: 'https://github.com/anthropics/hackathon-book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Module 1',
                to: '/docs/intro',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'ROS 2 Official Docs',
                href: 'https://docs.ros.org/en/humble/',
              },
              {
                label: 'GitHub Repository',
                href: 'https://github.com/anthropics/hackathon-book',
              },
            ],
          },
        ],
        copyright: `Copyright © 2025 AI-Spec-Driven Book Creation. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
        additionalLanguages: ['python', 'bash'],
      },
    }),
};

module.exports = config;
